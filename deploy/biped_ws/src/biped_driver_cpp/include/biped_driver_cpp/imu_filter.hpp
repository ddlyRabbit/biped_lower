/**
 * @file imu_filter.hpp
 * @brief 2nd-order Butterworth low-pass for IMU vectors (header-only, no ROS).
 *
 * Runs at the IMU report rate (~200 Hz) as an anti-aliasing / vibration
 * filter for the 50 Hz policy. Default 40 Hz cutoff is transparent
 * (≤ −0.6 dB) across the 0–25 Hz band the policy was trained on.
 */

#pragma once

#include <cmath>

namespace biped_driver_cpp {

/// 2nd-order Butterworth low-pass (RBJ biquad, Direct Form II transposed).
class ButterworthLPF {
public:
    /// cutoff_hz <= 0 or >= 0.45 * sample_hz → bypass (pass-through).
    void init(double cutoff_hz, double sample_hz) {
        z1_ = z2_ = 0.0;
        primed_ = false;
        if (cutoff_hz <= 0.0 || sample_hz <= 0.0 || cutoff_hz >= 0.45 * sample_hz) {
            bypass_ = true;
            return;
        }
        bypass_ = false;
        const double w0 = 2.0 * M_PI * cutoff_hz / sample_hz;
        const double q = 1.0 / std::sqrt(2.0);  // Butterworth
        const double alpha = std::sin(w0) / (2.0 * q);
        const double cw = std::cos(w0);
        const double a0 = 1.0 + alpha;
        b0_ = (1.0 - cw) / 2.0 / a0;
        b1_ = (1.0 - cw) / a0;
        b2_ = (1.0 - cw) / 2.0 / a0;
        a1_ = (-2.0 * cw) / a0;
        a2_ = (1.0 - alpha) / a0;
    }

    double filter(double x) {
        if (bypass_) return x;
        if (!primed_) {
            // Prime state at steady-state for input x (DC gain is 1) so the
            // output starts at the first input — no startup transient.
            z2_ = (b2_ - a2_) * x;
            z1_ = (b1_ - a1_) * x + z2_;
            primed_ = true;
        }
        const double y = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * y + z2_;
        z2_ = b2_ * x - a2_ * y;
        return y;
    }

    void reset() { z1_ = z2_ = 0.0; primed_ = false; }
    bool bypassed() const { return bypass_; }

private:
    double b0_ = 0, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
    double z1_ = 0, z2_ = 0;
    bool bypass_ = true;
    bool primed_ = false;
};

/// 3-axis wrapper.
class Vec3Filter {
public:
    void init(double cutoff_hz, double sample_hz) {
        for (auto& f : lpf_) f.init(cutoff_hz, sample_hz);
    }
    void apply(double v[3]) {
        v[0] = lpf_[0].filter(v[0]);
        v[1] = lpf_[1].filter(v[1]);
        v[2] = lpf_[2].filter(v[2]);
    }
    void reset() { for (auto& f : lpf_) f.reset(); }
    bool bypassed() const { return lpf_[0].bypassed(); }

private:
    ButterworthLPF lpf_[3];
};

}  // namespace biped_driver_cpp
