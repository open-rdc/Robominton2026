#!/usr/bin/env python3
"""End-to-end test. Source install/setup.bash first. Uses isolated ROS domain 87."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

# Set these BEFORE initializing ROS, so an existing robot is not addressed.
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='robom1-smoke-')

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def run_case(mode):
    log_path = Path(os.environ['ROS_LOG_DIR']) / (mode + '.log')
    with log_path.open('w') as log:
        process = subprocess.Popen(
            # We signal only the launch process during cleanup. Explicit noninteractive
            # mode makes launch forward SIGINT even when this test inherits a TTY.
            ['ros2', 'launch', '--noninteractive', 'robom1_bringup', 'sim.launch.py', 'rviz:=false',
             'demo:=' + ('true' if mode == 'demo' else 'false'),
             'follow_goal:=' + ('true' if mode == 'goal' else 'false'),
             'impact_demo:=' + ('true' if mode == 'impact' else 'false')],
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        node = rclpy.create_node('robom1_smoke_' + mode)
        received, positions, joints = {}, [], {}
        racket_angles = []
        racket_samples = []
        impact_received = {}
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        def record_odom(msg):
            received['odom'] = msg
            positions.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

        def record_joints(msg):
            joints.update(zip(msg.name, msg.position))
            if 'racket_swing_joint' in msg.name:
                racket_angles.append(joints['racket_swing_joint'])
                stamp_ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                racket_samples.append((stamp_ns, joints['racket_swing_joint']))

        subscriptions = [node.create_subscription(Odometry, 'odom', record_odom, 10),
                         node.create_subscription(JointState, 'joint_states', record_joints, 10)]
        for key, topic, cls in [('scan', 'scan', LaserScan), ('image', 'camera/image_raw', Image),
                                ('info', 'camera/camera_info', CameraInfo),
                                ('shuttle', 'shuttle/ground_truth', PointStamped)]:
            subscriptions.append(node.create_subscription(
                cls, topic, lambda msg, key=key: received.update({key: msg}), qos_profile_sensor_data))

        if mode == 'impact':
            for key, topic, cls in [
                    ('count', 'shuttle/hit_count', Int32),
                    ('outgoing', 'shuttle/impact_velocity', Vector3Stamped),
                    ('path', 'shuttle/post_hit_path', RosPath),
                    ('predicted', 'shuttle/predicted_landing', PointStamped),
                    ('actual', 'shuttle/actual_landing', PointStamped)]:
                subscriptions.append(node.create_subscription(
                    cls, topic, lambda msg, key=key: impact_received.update({key: msg}), 10))

        def spin(seconds, action=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if action:
                    action()
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            spin(3.0 if mode == 'impact' else 4.0)
            assert set(received) == {'odom', 'scan', 'image', 'info', 'shuttle'}, received.keys()
            assert len(joints) == 10, joints
            scan, image = received['scan'], received['image']
            assert len(scan.ranges) == 1441 and any(math.isfinite(v) for v in scan.ranges)
            assert len(image.data) == image.height * image.step == 320 * 240 * 3
            assert image.header.frame_id == 'camera_optical_frame'
            assert received['info'].k[0] == 220.0
            assert buffer.can_transform('odom', 'camera_optical_frame', rclpy.time.Time())
            transform = buffer.lookup_transform('base_footprint', 'lidar_link', rclpy.time.Time())
            assert abs(transform.transform.translation.z - 0.9) < 1e-6
            if mode == 'impact':
                assert set(impact_received) == {'count', 'outgoing', 'path', 'predicted', 'actual'}, impact_received.keys()
                assert impact_received['count'].data == 1
                assert abs(positions[-1][0]-1.0) < .01 and abs(positions[-1][1]) < .01
                assert impact_received['outgoing'].vector.x > 0 and impact_received['outgoing'].vector.z > 0
                assert len(impact_received['path'].poses) > 10
                head = buffer.lookup_transform('base_footprint', 'racket_head_link', rclpy.time.Time())
                transform_ns = head.header.stamp.sec*10**9 + head.header.stamp.nanosec
                sample_ns, angle = min(racket_samples, key=lambda sample: abs(sample[0]-transform_ns))
                assert abs(sample_ns-transform_ns) < 1e6
                expected = (0.05+0.62*math.sin(angle), -0.36, 1.05+0.62*math.cos(angle))
                translation = head.transform.translation
                assert all(abs(a-b) < .005 for a, b in zip(
                    (translation.x, translation.y, translation.z), expected)), (translation, expected, angle)
                predicted, actual = impact_received['predicted'].point, impact_received['actual'].point
                error = math.sqrt((predicted.x-actual.x)**2+(predicted.y-actual.y)**2+(predicted.z-actual.z)**2)
                assert error < .03, error
                assert abs(predicted.z-.035) < 1e-6
                # Incoming-feed service also takes a real collision path; it never
                # emits a fake hit directly from the service callback.
                feed = node.create_client(Trigger, 'shuttle/feed_to_racket')
                assert feed.wait_for_service(timeout_sec=2.)
                future = feed.call_async(Trigger.Request())
                spin(2.8)
                assert future.done() and future.result().success
                assert impact_received['count'].data == 1
            elif mode == 'demo':
                assert min(racket_angles) < -1.5 and max(racket_angles) > 1.7, racket_angles
                assert max(x for x, _ in positions) - min(x for x, _ in positions) > 0.3
            elif mode == 'manual':
                assert abs(positions[-1][0] - 1.0) < 0.01
                velocity = node.create_publisher(Twist, 'cmd_vel', 10)
                pitch = node.create_publisher(Float64, 'lidar_pitch/command', 10)
                racket = node.create_publisher(Float64, 'racket_swing/command', 10)
                spin(0.5)
                cmd = Twist()
                cmd.linear.y = 0.5
                spin(1.0, lambda: velocity.publish(cmd))
                assert positions[-1][1] > 0.25, positions[-1]
                cmd.linear.y, cmd.angular.z = 0.0, 0.5
                spin(1.0, lambda: velocity.publish(cmd))
                assert abs(received['odom'].pose.pose.orientation.z) > 0.1
                # Stop publishing entirely: watchdog plus acceleration ramp must stop the base.
                spin(1.0)
                stopped = received['odom'].twist.twist
                assert abs(stopped.linear.y) < 0.001 and abs(stopped.angular.z) < 0.001
                spin(0.8, lambda: (pitch.publish(Float64(data=0.4)), racket.publish(Float64(data=0.6))))
                assert abs(joints['lidar_pitch_joint'] - 0.4) < 0.01
                assert abs(joints['racket_swing_joint'] - 0.6) < 0.01
                spin(0.3)  # Drain the last manual angle messages before triggering a stroke.
                racket_angles.clear()
                trigger = node.create_client(Trigger, 'racket_swing/trigger')
                assert trigger.wait_for_service(timeout_sec=2.0)
                stroke = trigger.call_async(Trigger.Request())
                spin(3.2)
                assert stroke.done() and stroke.result().success
                assert min(racket_angles) < -1.5 and max(racket_angles) > 1.7
                assert abs(joints['racket_swing_joint'] + 0.35) < 0.01
                client = node.create_client(Trigger, 'shuttle/reset')
                assert client.wait_for_service(timeout_sec=2.0)
                future = client.call_async(Trigger.Request())
                spin(0.5)
                assert future.done() and future.result().success
            else:
                goal_pub = node.create_publisher(PoseStamped, 'goal_pose', 10)
                goal = PoseStamped()
                goal.header.frame_id = 'odom'
                goal.pose.position.x, goal.pose.position.y = 1.8, 0.6
                goal.pose.orientation.w = 1.0
                spin(3.0, lambda: goal_pub.publish(goal))
                assert math.hypot(positions[-1][0] - 1.8, positions[-1][1] - 0.6) < 0.1
        finally:
            node.destroy_node()
            if process.poll() is None:
                # Launch propagates SIGINT itself; signaling the group as well
                # delivers it twice and can interrupt normal node cleanup.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            errors = log_path.read_text()
            assert 'Traceback' not in errors and 'process has died' not in errors, errors
            del listener, subscriptions
        print('PASS:', mode, '(topics, joints, TF, behavior, shutdown)', flush=True)


if __name__ == '__main__':
    rclpy.init()
    try:
        for case in ['demo', 'manual', 'goal', 'impact']:
            run_case(case)
        print('Logs:', os.environ['ROS_LOG_DIR'])
    finally:
        rclpy.shutdown()
