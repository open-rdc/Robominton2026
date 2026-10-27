"""Small deterministic sensor sandbox, with analytic racket contacts; not a full rigid-body engine."""
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robom1_sim.swing import PitchSwing
from robom1_sim.physics import (RacketGeometry, collide_racket, flight_step,
                               make_impact_feed, predict_landing, racket_pose)
from robom1_sim.geometry import project, rotation_up, rotation_z, rotation_rpy, scan_scene


class SceneSim(Node):
    def __init__(self):
        super().__init__('scene_sim')
        self.demo_aim = self.declare_parameter('demo_aim', True).value
        self.impact_demo = self.declare_parameter('impact_demo', False).value
        self.demo_swing = self.declare_parameter('demo_swing', True).value and not self.impact_demo
        self.restitution = float(self.declare_parameter('racket_restitution', 0.8).value)
        self.racket = RacketGeometry(**{
            name: float(self.declare_parameter('racket_' + name, value).value)
            for name, value in vars(RacketGeometry()).items()
        })
        if not 0.0 <= self.restitution <= 1.0:
            raise ValueError('racket_restitution must be in [0, 1]')
        if not all(math.isfinite(v) for v in vars(self.racket).values()):
            raise ValueError('Racket geometry must be finite')
        if min(self.racket.reach, self.racket.radius, self.racket.thickness) <= 0:
            raise ValueError('Racket dimensions must be positive')
        self.period = float(self.declare_parameter('serve_period', 4.0).value)
        self.drag = float(self.declare_parameter('shuttle_drag', 0.12).value)
        self.radius = float(self.declare_parameter('shuttle_radius', 0.035).value)
        if self.period <= 0 or self.drag < 0 or self.radius <= 0:
            raise ValueError('serve_period/radius must be positive and drag nonnegative')
        self.camera = {}
        self.lidar = {}
        camera_defaults = dict(xyz=[0.12, 0.0, 1.0], rpy=[0.0, 0.0, 0.0], width=320,
                               height=240, fx=220.0, fy=220.0, cx=160.0, cy=120.0, rate=10.0)
        lidar_defaults = dict(xyz=[0.18, 0.0, 0.9], rpy=[0.0, 0.0, 0.0], lower=-0.6,
                              upper=1.2, initial=0.0, velocity=1.5, angle_min=-math.pi,
                              angle_max=math.pi, samples=1441, range_min=0.05, range_max=20.0, rate=10.0)
        for name, defaults, target in [('camera', camera_defaults, self.camera),
                                       ('lidar', lidar_defaults, self.lidar)]:
            for key, default in defaults.items():
                target[key] = self.declare_parameter(name+'_'+key, default).value
        self.camera_mount = rotation_rpy(self.camera['rpy'])
        self.lidar_mount = rotation_rpy(self.lidar['rpy'])
        self.camera_age = self.scan_age = 0.0
        self.base = np.array([1.0, 0.0, 0.0])
        self.yaw = 0.0
        self.have_odom = False
        self.base_velocity = np.zeros(3)
        self.yaw_rate = 0.0
        self.previous_base = self.base.copy()
        self.previous_yaw = self.yaw
        defaults = PitchSwing()
        swing_defaults = dict(lower=defaults.lower, upper=defaults.upper, ready=defaults.ready,
                              backswing=defaults.backswing, follow_through=defaults.follow_through,
                              velocity=defaults.max_speed, manual_velocity=defaults.manual_velocity,
                              preparation_time=defaults.preparation_time, stroke_time=defaults.stroke_time,
                              hold_time=defaults.hold_time, return_time=defaults.return_time)
        self.stroke = PitchSwing(**{key: float(self.declare_parameter('swing_'+key, value).value)
                                    for key, value in swing_defaults.items()})
        self.pitch = self.pitch_target = self.lidar['initial']
        self.swing = self.swing_target = self.stroke.ready
        self.auto_swung = False
        self.age = self.output_age = 0.0
        self.serve_count = 0
        self.shuttle = np.zeros(3)
        self.velocity = np.zeros(3)
        self.reset_shuttle()
        self.last = self.get_clock().now()
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Float64, 'lidar_pitch/command', self.on_pitch, 10)
        self.create_subscription(Float64, 'racket_swing/command', self.on_swing, 10)
        self.create_service(Trigger, 'shuttle/reset', self.on_reset)
        self.create_service(Trigger, 'racket_swing/trigger', self.on_trigger_swing)
        self.create_service(Trigger, 'shuttle/feed_to_racket', self.on_feed)
        self.joints_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.truth_pub = self.create_publisher(PointStamped, 'shuttle/ground_truth', 10)
        self.velocity_pub = self.create_publisher(Vector3Stamped, 'shuttle/velocity', 10)
        self.hit_count_pub = self.create_publisher(Int32, 'shuttle/hit_count', 10)
        self.impact_pub = self.create_publisher(PointStamped, 'shuttle/impact_point', 10)
        self.outgoing_pub = self.create_publisher(Vector3Stamped, 'shuttle/impact_velocity', 10)
        self.path_pub = self.create_publisher(Path, 'shuttle/post_hit_path', 10)
        self.landing_pub = self.create_publisher(PointStamped, 'shuttle/predicted_landing', 10)
        self.landing_time_pub = self.create_publisher(Float64, 'shuttle/predicted_flight_time', 10)
        self.actual_landing_pub = self.create_publisher(PointStamped, 'shuttle/actual_landing', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'sim/markers', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.angles = np.linspace(self.lidar['angle_min'], self.lidar['angle_max'], self.lidar['samples'])
        self.rays = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros(len(self.angles))))
        self.create_timer(0.02, self.step)
        self.get_logger().info('Sensors and moving-racket collision ready; predictions use simplified drag.')

    def on_odom(self, msg):
        self.base[:2] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        first = not self.have_odom
        self.have_odom = True
        self.base_velocity = rotation_z(self.yaw) @ np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, 0.0])
        self.yaw_rate = msg.twist.twist.angular.z
        if first:
            self.previous_base = self.base.copy()
            self.previous_yaw = self.yaw
            if self.impact_demo:
                self.reset_shuttle()

    def on_pitch(self, msg):
        if math.isfinite(msg.data):
            self.pitch_target = float(np.clip(msg.data, self.lidar['lower'], self.lidar['upper']))
            # A manual pitch command takes over from automatic demo aiming.
            self.demo_aim = False

    def on_swing(self, msg):
        if math.isfinite(msg.data):
            self.demo_swing = False
            self.stroke.cancel(self.swing)
            self.swing_target = float(np.clip(msg.data, self.stroke.lower, self.stroke.upper))

    def on_trigger_swing(self, request, response):
        del request
        response.success = self.stroke.trigger()
        response.message = 'Pitch stroke started.' if response.success else 'Stroke already in progress.'
        return response

    def reset_shuttle(self, force_feed=False):
        self.age = 0.0
        self.auto_swung = False
        side = 1.0 if self.serve_count % 2 == 0 else -1.0
        self.shuttle[:] = [5.0, -side * 0.7, 1.8]
        self.velocity[:] = [-1.6, side * 0.65, 4.8]
        self.serve_count += 1
        self.hit_count = 0
        self.impact_point = None
        self.prediction = None
        self.predicted_landing = None
        self.actual_landing = None
        self.flight_time = None
        self.impact_speed = 0.0
        self.flight_history = [self.shuttle.copy()]
        if (self.impact_demo or force_feed) and self.have_odom:
            self.stroke.cancel(self.stroke.ready)
            self.swing = self.swing_target = self.stroke.ready
            self.shuttle[:], self.velocity[:] = make_impact_feed(
                self.base, self.yaw, self.racket, self.radius, self.drag, self.stroke)
            self.flight_history = [self.shuttle.copy()]
            self.stroke.trigger()
            self.auto_swung = True
        if hasattr(self, 'path_pub'):
            empty = Path()
            empty.header.frame_id = 'odom'
            empty.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty)

    def on_feed(self, request, response):
        del request
        if not self.have_odom or np.linalg.norm(self.base_velocity) > 0.01 or abs(self.yaw_rate) > 0.01:
            response.success = False
            response.message = 'Stop the base before starting the stationary-base impact feed.'
            return response
        self.reset_shuttle(force_feed=True)
        response.success = True
        response.message = 'Incoming test feed and pitch stroke started; collision decides the hit.'
        return response

    def on_reset(self, request, response):
        del request
        self.reset_shuttle()
        response.success = True
        response.message = 'Shuttle relaunched; base pose is unchanged.'
        return response

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if dt <= 0 or dt > 0.5 or not self.have_odom:
            return
        self.age += dt
        if self.age >= self.period:
            self.reset_shuttle()
        if self.demo_swing and not self.auto_swung and self.age >= 0.3:
            self.auto_swung = self.stroke.trigger()
        # Resolve both flight and moving-racket contact in small steps. Limit the
        # relative travel to 0.4 sphere radii to avoid crossing the thin face unseen.
        relative_speed = (np.linalg.norm(self.velocity) + np.linalg.norm(self.base_velocity)
                          + self.stroke.max_speed * self.racket.reach
                          + abs(self.yaw_rate) * (np.linalg.norm(self.racket.pivot) + self.racket.reach))
        max_step = min(0.002, 0.4 * self.radius / max(1.0, relative_speed))
        count = max(1, math.ceil(dt / max_step))
        h = dt / count
        yaw_delta = math.atan2(math.sin(self.yaw-self.previous_yaw), math.cos(self.yaw-self.previous_yaw))
        for index in range(count):
            previous_swing = self.swing
            if self.stroke.active:
                self.swing = self.stroke.step(h)
                self.swing_target = self.swing
            else:
                self.swing += float(np.clip(self.swing_target-self.swing, -self.stroke.manual_velocity*h, self.stroke.manual_velocity*h))
                self.stroke.position = self.swing
            pitch_rate = (self.swing - previous_swing) / h
            if self.shuttle[2] <= self.radius:
                continue
            old_position = self.shuttle.copy()
            self.shuttle, self.velocity = flight_step(self.shuttle, self.velocity, h, self.drag)
            fraction = (index + 1) / count
            base = self.previous_base + fraction * (self.base - self.previous_base)
            yaw = self.previous_yaw + fraction * yaw_delta
            self.shuttle, self.velocity, impact = collide_racket(
                self.shuttle, self.velocity, self.radius, base, yaw, self.swing,
                self.base_velocity, self.yaw_rate, pitch_rate, self.racket, self.restitution)
            event_ns = now.nanoseconds - int((dt - (index+1)*h)*1e9)
            if impact is not None:
                self.record_impact(impact, event_ns)
            if self.shuttle[2] <= self.radius:
                fraction = (old_position[2]-self.radius) / (old_position[2]-self.shuttle[2])
                self.shuttle = old_position + fraction * (self.shuttle-old_position)
                self.shuttle[2] = self.radius
                self.velocity[:] = 0.0
                self.actual_landing = self.shuttle.copy()
                self.publish_point(self.actual_landing_pub, self.actual_landing,
                                   rclpy.time.Time(nanoseconds=event_ns).to_msg())
        self.previous_base = self.base.copy()
        self.previous_yaw = self.yaw
        rotation = rotation_z(self.yaw)
        if self.demo_aim:
            local = self.lidar_mount.T @ (rotation.T @ (self.shuttle-self.base) - self.lidar['xyz'])
            self.pitch_target = float(np.clip(math.atan2(local[2], local[0]), self.lidar['lower'], self.lidar['upper']))
        self.pitch += float(np.clip(self.pitch_target - self.pitch, -self.lidar['velocity']*dt, self.lidar['velocity']*dt))
        stamp = now.to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['lidar_pitch_joint', 'racket_swing_joint']
        joints.position = [self.pitch, self.swing]
        self.joints_pub.publish(joints)
        truth = PointStamped()
        truth.header.stamp, truth.header.frame_id = stamp, 'odom'
        truth.point = Point(x=float(self.shuttle[0]), y=float(self.shuttle[1]), z=float(self.shuttle[2]))
        self.truth_pub.publish(truth)
        speed = Vector3Stamped()
        speed.header = truth.header
        speed.vector.x, speed.vector.y, speed.vector.z = map(float, self.velocity)
        self.velocity_pub.publish(speed)
        self.hit_count_pub.publish(Int32(data=self.hit_count))
        self.camera_age += dt
        self.scan_age += dt
        if self.camera_age >= 1.0/self.camera['rate']:
            self.camera_age %= 1.0/self.camera['rate']
            self.publish_camera(stamp, rotation)
        if self.scan_age >= 1.0/self.lidar['rate']:
            self.scan_age %= 1.0/self.lidar['rate']
            self.publish_scan(stamp, rotation)
        self.output_age += dt
        if self.output_age >= 0.1:
            self.output_age %= 0.1
            self.flight_history.append(self.shuttle.copy())
            self.publish_markers(stamp)

    @staticmethod
    def publish_point(publisher, point, stamp):
        msg = PointStamped()
        msg.header.frame_id, msg.header.stamp = 'odom', stamp
        msg.point = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        publisher.publish(msg)

    def record_impact(self, point, event_ns):
        self.hit_count += 1
        self.impact_point = point.copy()
        self.impact_speed = float(np.linalg.norm(self.velocity))
        self.prediction, times, self.predicted_landing = predict_landing(
            self.shuttle, self.velocity, self.radius, self.drag)
        self.flight_time = float(times[-1]) if self.predicted_landing is not None else None
        stamp = rclpy.time.Time(nanoseconds=event_ns).to_msg()
        self.publish_point(self.impact_pub, point, stamp)
        outgoing = Vector3Stamped()
        outgoing.header.frame_id, outgoing.header.stamp = 'odom', stamp
        outgoing.vector.x, outgoing.vector.y, outgoing.vector.z = map(float, self.velocity)
        self.outgoing_pub.publish(outgoing)
        path = Path()
        path.header = outgoing.header
        indices = list(range(0, len(times)-1, 10)) + [len(times)-1]
        for index in indices:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = rclpy.time.Time(nanoseconds=event_ns+int(times[index]*1e9)).to_msg()
            p = self.prediction[index]
            pose.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        if self.predicted_landing is not None:
            self.publish_point(self.landing_pub, self.predicted_landing, stamp)
            self.landing_time_pub.publish(Float64(data=self.flight_time))
        self.get_logger().info(
            'Racket hit #{}, outgoing {:.2f} m/s, predicted landing {}'.format(
                self.hit_count, self.impact_speed,
                np.round(self.predicted_landing, 3).tolist() if self.predicted_landing is not None else 'outside horizon'))

    def publish_scan(self, stamp, rotation):
        origin = self.base + rotation @ np.asarray(self.lidar['xyz'])
        directions = self.rays @ (rotation @ self.lidar_mount @ rotation_up(self.pitch)).T
        ranges = scan_scene(origin, directions, self.shuttle, self.radius, self.lidar['range_max'], self.lidar['range_min'])
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'lidar_link'
        scan.angle_min, scan.angle_max = self.lidar['angle_min'], self.lidar['angle_max']
        scan.angle_increment = (self.lidar['angle_max']-self.lidar['angle_min']) / (len(self.angles)-1)
        scan.scan_time, scan.time_increment = 1.0/self.lidar['rate'], 0.0  # instantaneous scan, no rolling acquisition
        scan.range_min, scan.range_max = self.lidar['range_min'], self.lidar['range_max']
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(scan)

    def publish_camera(self, stamp, rotation):
        width, height = self.camera['width'], self.camera['height']
        fx, fy, cx, cy = [self.camera[key] for key in ['fx', 'fy', 'cx', 'cy']]
        origin = self.base + rotation @ np.asarray(self.camera['xyz'])
        local = self.camera_mount.T @ rotation.T @ (self.shuttle - origin)
        frame = np.full((height, width, 3), (35, 45, 25), dtype=np.uint8)
        # Synthetic pinhole image of the shuttle only; no real image recognition here.
        pixel = project(local, fx, fy, cx, cy)
        if pixel is not None:
            u, v = pixel
            radius = max(1, int(min(fx, fy) * self.radius / local[0]))
            if -radius <= u < width + radius and -radius <= v < height + radius:
                cv2.circle(frame, (int(round(float(u))), int(round(float(v)))), radius, (245, 245, 245), -1)
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, 'camera_optical_frame'
        image.height, image.width = height, width
        image.encoding, image.is_bigendian, image.step = 'bgr8', 0, width * 3
        image.data = frame.tobytes()
        self.image_pub.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = height, width
        info.distortion_model, info.d = 'plumb_bob', [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def publish_markers(self, stamp):
        markers = MarkerArray()

        def marker(identifier, kind, position, scale, color):
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.ns, m.id, m.type, m.action = 'badminton', identifier, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
            m.scale.x, m.scale.y, m.scale.z = map(float, scale)
            m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
            markers.markers.append(m)
            return m

        marker(0, Marker.CUBE, (6.7, 0, -0.025), (13.4, 6.1, 0.04), (0.06, 0.3, 0.2, 1))
        lines = marker(1, Marker.LINE_LIST, (0, 0, 0), (0.035, 0, 0), (1, 1, 1, 1))
        for x in [0.0, 0.76, 4.72, 6.7, 8.68, 12.64, 13.4]:
            lines.points.extend([Point(x=x, y=-3.05, z=0.01), Point(x=x, y=3.05, z=0.01)])
        for y in [-3.05, -2.59, 2.59, 3.05]:
            lines.points.extend([Point(x=0.0, y=y, z=0.01), Point(x=13.4, y=y, z=0.01)])
        for a, b in [(0.0, 4.72), (8.68, 13.4)]:
            lines.points.extend([Point(x=a, y=0.0, z=0.01), Point(x=b, y=0.0, z=0.01)])
        marker(2, Marker.CUBE, (6.7, 0, 1.15), (0.02, 6.1, 0.75), (0.8, 0.8, 0.8, 0.4))
        marker(3, Marker.SPHERE, self.shuttle, (2*self.radius,)*3, (1, 1, 0.9, 1))
        marker(4, Marker.CYLINDER, (6.7, -3.05, 0.775), (0.04, 0.04, 1.55), (0.9, 0.9, 0.9, 1))
        marker(5, Marker.CYLINDER, (6.7, 3.05, 0.775), (0.04, 0.04, 1.55), (0.9, 0.9, 0.9, 1))
        label = marker(6, Marker.TEXT_VIEW_FACING, (3.0, 0, 3.5), (0, 0, 0.18), (1, 1, 1, 1))
        label.text = 'Racket hits: {} | outgoing: {:.1f} m/s'.format(self.hit_count, self.impact_speed)
        if self.flight_time is not None:
            label.text += '\nFlight {:.2f}s | landing ({:.2f}, {:.2f}) m'.format(
                self.flight_time, self.predicted_landing[0], self.predicted_landing[1])
        # Head-center arc in the moving base frame, matching the grip-end URDF pivot.
        arc = marker(7, Marker.LINE_STRIP, (0, 0, 0), (0.012, 0, 0), (0.1, 0.9, 1, 0.65))
        arc.header.frame_id = 'base_footprint'
        for angle in np.linspace(self.stroke.backswing, self.stroke.follow_through, 81):
            _, center, _ = racket_pose(np.zeros(3), 0.0, angle, self.racket)
            arc.points.append(Point(x=float(center[0]), y=float(center[1]), z=float(center[2])))
        history = marker(8, Marker.LINE_STRIP, (0, 0, 0), (0.018, 0, 0), (0.2, 0.5, 1, 1))
        history.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.flight_history]
        if self.prediction is not None:
            forecast = marker(9, Marker.LINE_STRIP, (0, 0, 0), (0.025, 0, 0), (0.2, 1, 0.3, 1))
            forecast.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.prediction[::5]]
            marker(10, Marker.SPHERE, self.impact_point, (0.09,)*3, (1, 0.15, 0.05, 1))
        else:
            for identifier in (9, 10):
                marker(identifier, Marker.SPHERE, (0, 0, 0), (0.01,)*3, (1, 1, 1, 1)).action = Marker.DELETE
        if self.predicted_landing is not None:
            landing_marker = self.predicted_landing.copy()
            landing_marker[2] = 0.025
            marker(11, Marker.CYLINDER, landing_marker, (0.25, 0.25, 0.04), (1, 0.85, 0, 1))
        else:
            marker(11, Marker.CYLINDER, (0, 0, 0), (0.01,)*3, (1, 1, 1, 1)).action = Marker.DELETE
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SceneSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
