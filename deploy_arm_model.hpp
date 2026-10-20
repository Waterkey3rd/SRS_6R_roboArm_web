#pragma once
#include <array>
// Engineer2026: +X forward, +Y left, +Z up. Limits from ArmMiddleware.cpp.
namespace deploy_arm_model {
inline constexpr int kJointCount = 6;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kExtra = 15.0 * kPi / 180.0;
inline constexpr std::array<double, 6> lower = {-240 * kPi / 180,
                                                -3.14 / 4 * 3 - 10 * kPi / 180 -
                                                    kExtra,
                                                -.6 - kExtra,
                                                -12.6 * 1000 - kExtra,
                                                -130 * kPi / 180 + kExtra,
                                                -6.28 * 1000 - kExtra};
inline constexpr std::array<double, 6> upper = {
    240 * kPi / 180,      3.14 / 4 + .3 + kExtra,   3.14 / 2 + .5 + kExtra,
    12.6 * 1000 + kExtra, 130 * kPi / 180 + kExtra, 6.28 * 1000 + kExtra};
// Historical ABI slot 1 (right) now denotes the sole arm.
inline bool validSide(int side) { return side == 1; }
} // namespace deploy_arm_model
