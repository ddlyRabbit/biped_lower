/**
 * @file ankle_linkage.hpp
 * @brief Parallel linkage transform for ankle joints.
 *
 * Two RS02 motors per ankle, U-joint to foot plate.
 * Forward: joint-space (pitch, roll) → motor-space (upper, lower)
 * Inverse: motor-space (upper, lower) → joint-space (pitch, roll)
 *
 * From Onshape CAD:
 *   K_P = d/r   = 41.14/32.249  = 1.2757
 *   K_R = c/2/r = 31.398/32.249 = 0.9736
 */

#pragma once

#include <utility>

namespace biped_driver_cpp {

// Linkage constants
constexpr double PITCH_GAIN = 41.14 / 32.249;          // K_P = 1.2757
constexpr double ROLL_GAIN  = 31.398 / 32.249;         // K_R = 0.9736
constexpr double INV_PITCH  = 32.249 / (2.0 * 41.14);  // 1/(2*K_P) = 0.3919
constexpr double INV_ROLL   = 32.249 / (2.0 * 31.398); // 1/(2*K_R) = 0.5136

/**
 * Forward: joint-space (pitch, roll) → motor-space (upper, lower).
 *
 *   motor_A (upper) =  pitch_sign × K_P × pitch − K_R × roll
 *   motor_B (lower) = −pitch_sign × K_P × pitch − K_R × roll
 *
 * @param pitch_sign +1 for R ankle, −1 for L ankle (mirrored pitch axis)
 */
inline std::pair<double, double> ankle_command_to_motors(
    double pitch_cmd, double roll_cmd, int pitch_sign = 1) {
    double motor_upper =  pitch_sign * PITCH_GAIN * pitch_cmd - ROLL_GAIN * roll_cmd;
    double motor_lower = -pitch_sign * PITCH_GAIN * pitch_cmd - ROLL_GAIN * roll_cmd;
    return {motor_upper, motor_lower};
}

/**
 * Inverse: motor-space (upper, lower) → joint-space (pitch, roll).
 *
 *   pitch =  pitch_sign × INV_P × (A − B)
 *   roll  = −INV_R × (A + B)
 *
 * @param pitch_sign +1 for R ankle, −1 for L ankle
 */
inline std::pair<double, double> ankle_motors_to_feedback(
    double upper_pos, double lower_pos, int pitch_sign = 1) {
    double foot_pitch =  pitch_sign * INV_PITCH * (upper_pos - lower_pos);
    double foot_roll  = -INV_ROLL * (upper_pos + lower_pos);
    return {foot_pitch, foot_roll};
}

}  // namespace biped_driver_cpp
