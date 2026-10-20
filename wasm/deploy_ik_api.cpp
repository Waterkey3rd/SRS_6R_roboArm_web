#include "../../App/Arm/Lib/myfkine26pro/myfkine26pro.h"
#include "../../App/Arm/Lib/myikine26pro/myikine26pro.h"
#include "../../App/Arm/Lib/smartikine26pro/smartikine26pro.h"
#include "../Arm/Inc/Middleware/TrajectoryPlannerKernel.hpp"
#include "../deploy_arm_model.hpp"
#include <algorithm>
#include <cmath>
#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#define WEB_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define WEB_EXPORT extern "C"
#endif
namespace {
bool finite(const double *v, int n) {
  if (!v)
    return false;
  for (int i = 0; i < n; ++i)
    if (!std::isfinite(v[i]))
      return false;
  return true;
}
void rotation(double yaw, double pitch, double roll, double *r) {
  const auto cy = cos(yaw), sy = sin(yaw), cp = cos(pitch), sp = sin(pitch),
             cr = cos(roll), sr = sin(roll);
  const double m[9] = {cy * cp,
                       cy * sp * sr - sy * cr,
                       cy * sp * cr + sy * sr,
                       sy * cp,
                       sy * sp * sr + cy * cr,
                       sy * sp * cr - cy * sr,
                       -sp,
                       cp * sr,
                       cp * cr};
  std::copy_n(m, 9, r);
}
void fk(const double *q, double *r, double *p) {
  float x, y, z, yaw, pitch, roll;
  myfkine26pro(q[0], q[1], q[2], q[3], q[4], q[5], &x, &y, &z, &yaw, &pitch,
               &roll);
  p[0] = x;
  p[1] = y;
  p[2] = z;
  rotation(yaw, pitch, roll, r);
}
} // namespace
WEB_EXPORT int deploy_model_version() { return 2; }
WEB_EXPORT int deploy_fk(int side, const double *q, double *r, double *p) {
  if (!deploy_arm_model::validSide(side) || !finite(q, 6) || !r || !p)
    return 0;
  fk(q, r, p);
  return 1;
}
WEB_EXPORT int deploy_ik_solve(int side, const double *r, const double *p,
                               const double *current, double *out,
                               double *diag) {
  if (!deploy_arm_model::validSide(side) || !finite(r, 9) || !finite(p, 3) ||
      !finite(current, 6) || !out || !diag)
    return 0;
  std::fill_n(diag, 8, 0.0);
  const float pitch = atan2(-r[6], hypot(r[0], r[3]));
  const float yaw =
      hypot(r[0], r[3]) > 1e-6 ? atan2(r[3], r[0]) : atan2(-r[1], r[4]);
  const float roll = hypot(r[0], r[3]) > 1e-6 ? atan2(r[7], r[8]) : 0;
  float q[6]{};
  unsigned char missing = 0;
  smartikine26pro(p[0], p[1], p[2], yaw, pitch, roll, current[0], current[1],
                  current[2], current[3], current[4], current[5], &missing,
                  &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]);
  if (missing) {
    myikine26pro(p[0], p[1], p[2], yaw, pitch, roll, &missing, &q[0], &q[1],
                 &q[2], &q[3], &q[4], &q[5]);
    diag[3] = 1;
  }
  bool ok = !missing;
  for (int i = 0; i < 6; ++i) {
    out[i] = current[i] + std::remainder(double(q[i]) - current[i],
                                         2 * deploy_arm_model::kPi);
    ok = ok && std::isfinite(out[i]) &&
         out[i] >= deploy_arm_model::lower[i] - 2e-6 &&
         out[i] <= deploy_arm_model::upper[i] + 2e-6;
  }
  double actualR[9], actualP[3];
  fk(out, actualR, actualP);
  double error = 0, trace = 0;
  for (int i = 0; i < 3; ++i)
    error += (p[i] - actualP[i]) * (p[i] - actualP[i]);
  for (int i = 0; i < 9; ++i)
    trace += r[i] * actualR[i];
  diag[1] = sqrt(error);
  diag[2] = acos(std::clamp((trace - 1) / 2, -1.0, 1.0));
  ok = ok && diag[1] < 1 && diag[2] < .01;
  diag[0] = ok;
  return ok;
}
// Modified DH from myfkine26pro.c: six joint origins + TCP, six axes.
WEB_EXPORT int deploy_fk_chain(int side, const double *q, double *positions,
                               double *axes) {
  if (!deploy_arm_model::validSide(side) || !finite(q, 6) || !positions ||
      !axes)
    return 0;
  const double pi = deploy_arm_model::kPi;
  const double alpha[6] = {0, pi / 2, 0, pi / 2, -pi / 2, pi / 2},
               a[6] = {0, 0, 300, 0, 0, 0}, d[6] = {0, 0, 0, 266, 0, 174},
               offset[6] = {0, 3 * pi / 4, -pi / 4, 0, 0, 0};
  double t[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  for (int i = 0; i < 6; ++i) {
    const auto ca = cos(alpha[i]), sa = sin(alpha[i]),
               ct = cos(q[i] + offset[i]), st = sin(q[i] + offset[i]);
    for (int row = 0; row < 3; ++row) {
      positions[3 * i + row] = t[4 * row + 3] + t[4 * row] * a[i];
      axes[3 * i + row] = -sa * t[4 * row + 1] + ca * t[4 * row + 2];
    }
    const double m[16] = {ct,  -st,        0,       a[i],    st * ca, ct * ca,
                          -sa, -sa * d[i], st * sa, ct * sa, ca,      ca * d[i],
                          0,   0,          0,       1};
    double next[16]{};
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        for (int k = 0; k < 4; ++k)
          next[4 * row + col] += t[4 * row + k] * m[4 * k + col];
    std::copy_n(next, 16, t);
  }
  for (int row = 0; row < 3; ++row)
    positions[18 + row] = t[4 * row + 3];
  return 1;
}
WEB_EXPORT int deploy_get_joint_limits(int side, double *lo, double *hi) {
  if (!deploy_arm_model::validSide(side) || !lo || !hi)
    return 0;
  std::copy_n(deploy_arm_model::lower.begin(), 6, lo);
  std::copy_n(deploy_arm_model::upper.begin(), 6, hi);
  return 1;
}
// Offline quintic preview; MCU uses its own feedback-dependent tracking
// planner.
WEB_EXPORT double deploy_trajectory_duration(const double *start,
                                             const double *target,
                                             double requestedMs) {
  if (!finite(start, 6) || !finite(target, 6) || !std::isfinite(requestedMs))
    return 0;
  float a[6], b[6];
  for (int i = 0; i < 6; ++i) {
    a[i] = start[i];
    b[i] = target[i];
  }
  return 1000.0 *
         trajectory_planner_kernel::synchronizedDurationSeconds(
             a, b, 6, requestedMs * .001,
             trajectory_planner_kernel::kDefaultSpeedMaxRadPerSec,
             trajectory_planner_kernel::kDefaultAccelerationMaxRadPerSec2, .5f,
             8.f, .00001f);
}
WEB_EXPORT int deploy_trajectory_sample(const double *start,
                                        const double *target, double durationMs,
                                        double elapsedMs, double *p,
                                        double *v) {
  if (!finite(start, 6) || !finite(target, 6) || !p || !v ||
      !std::isfinite(durationMs) || !std::isfinite(elapsedMs))
    return 0;
  const float duration = durationMs * .001f, elapsed = elapsedMs * .001f;
  for (int i = 0; i < 6; ++i) {
    p[i] = trajectory_planner_kernel::samplePosition(start[i], target[i],
                                                     elapsed, duration);
    v[i] = duration > 1e-6f ? (target[i] - start[i]) / duration *
                                  trajectory_planner_kernel::normalizedVelocity(
                                      elapsed / duration)
                            : 0;
  }
  return 1;
}
