#pragma once
#include <cmath>

namespace robom1_control
{
constexpr double pi = 3.14159265358979323846;
inline double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
struct Wheel {double steering; double velocity;};
// Wheel position and velocity are expressed in base_link (x forward, y left).
// Keep steering travel <= 90 degrees by reversing the wheel if necessary.
inline Wheel inverse(double vx, double vy, double wz, double x, double y,
  double radius, double previous)
{
  const double wx = vx - wz * y, wy = vy + wz * x;
  double speed = std::hypot(wx, wy);
  if (speed < 1e-9) {return {previous, 0.0};}
  double delta = wrap(std::atan2(wy, wx) - previous);
  if (std::abs(delta) > pi / 2) {
    delta = wrap(delta + pi);
    speed = -speed;
  }
  return {previous + delta, speed / radius};
}
// Exact constant body-twist integration, including simultaneous translation/rotation.
inline void integrate(double & x, double & y, double & yaw,
  double vx, double vy, double wz, double dt)
{
  double dx = vx * dt, dy = vy * dt;
  if (std::abs(wz) > 1e-8) {
    const double a = std::sin(wz * dt) / wz;
    const double b = (1.0 - std::cos(wz * dt)) / wz;
    dx = a * vx - b * vy;
    dy = b * vx + a * vy;
  }
  x += std::cos(yaw) * dx - std::sin(yaw) * dy;
  y += std::sin(yaw) * dx + std::cos(yaw) * dy;
  yaw = wrap(yaw + wz * dt);
}
}  // namespace robom1_control
