//
// Created by Ao Shen on 19-6-15
//

#ifndef PUBLICCOURSE_SAO_HELPERS_H
#define PUBLICCOURSE_SAO_HELPERS_H

#include <cmath>

#include "common/proto/geometry.pb.h"
#include "common/proto/transform.pb.h"
#include "common/utils/math/math_utils.h"

namespace sao_agent {

class PID {
  static constexpr double epsilon = 0.01;

 public:
  PID(double imax, double imin, double dt_)
      : imax_(imax), imin_(imin), dt(dt_) {}

  void Tone(double p_, double i_, double d_) {
    P = p_;
    I = i_;
    D = d_;
  }

  double SetGoal(double new_goal) {
    goal_ = new_goal;
    return goal_;
  }

  double RunOneIteration(double current) {
    current_ = current;
    auto error = error_();
    double derivative;

    if (error > epsilon) integral_ = integral_ + error * dt;
    if (integral_ > imax_)
      integral_ = imax_;
    else if (integral_ < imin_)
      integral_ = imin_;
    derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    return (P * error + I * integral_ + D * derivative);
  }

 private:
  double error_() { return goal_ - current_; }
  double current_, goal_, prev_error_ = 0.0, integral_ = 0.0;
  double P = 0.0, I, D;
  const double imax_, imin_, dt;
};

inline interface::geometry::Point2D Point2D(
    const interface::geometry::Point3D& point) {
  interface::geometry::Point2D res;
  res.set_x(point.x());
  res.set_y(point.y());
  return res;
}
inline interface::geometry::Point2D Point2D(double x, double y) {
  interface::geometry::Point2D res;
  res.set_x(x);
  res.set_y(y);
  return res;
}
inline math::Vec3d Vec3d(const interface::geometry::Vector3d& p) {
  return math::Vec3d(p.x(), p.y(), p.z());
}

inline double Norm(double x, double y) { return std::sqrt(x * x + y * y); }
inline double Norm(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}
inline double CalcVelocity(const interface::geometry::Vector3d& v) {
  return Norm(v.x(), v.y());
}

inline double CrossProd(const interface::geometry::Vector3d& v,
                        const math::Vec3d& u) {
  double u_length = Norm(u.x, u.y, u.z);
  double v_length = Norm(v.x(), v.y(), v.z());
  if (u_length == 0.0 || v_length == 0.0) return 0.0;
  return (v.x() * u.y - u.x * v.y()) / (u_length * v_length);
}
inline double Distance(const interface::geometry::Vector3d& u,
                       const interface::geometry::Point2D& v) {
  return Norm(u.x() - v.x(), u.y() - v.y());
}
inline double Distance(const interface::geometry::Vector3d& u,
                       const interface::geometry::Point3D& v) {
  return Norm(u.x() - v.x(), u.y() - v.y());
}
inline math::Vec3d Center(const std::vector<interface::geometry::Point3D>& ps) {
  int sz = ps.size();
  double x = 0.0, y = 0.0;
  for (const auto& p : ps) {
    x += p.x();
    y += p.y();
  }
  return math::Vec3d(x / sz, y / sz, 0.0);
}
}  // namespace sao_agent

#endif  // PUBLICCOURSE_SAO_HELPERS_H
