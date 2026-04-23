#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <array>

namespace biped_control_cpp {

extern const std::vector<std::string> ISAAC_JOINT_ORDER;
extern const std::vector<std::string> JOINT_ORDER;
extern const std::vector<std::string> HIP_POS_ORDER;
extern const std::vector<std::string> KNEE_POS_ORDER;
extern const std::vector<std::string> FOOT_PITCH_ORDER;
extern const std::vector<std::string> FOOT_ROLL_ORDER;
extern const std::vector<std::string> ACTION_ORDER;
extern const std::vector<std::string> CSV_JOINT_ORDER;

// C++ standard guarantees initialization order for primitives, but we must populate these at runtime
extern std::unordered_map<std::string, double> DEFAULT_POSITIONS;
extern std::unordered_map<std::string, std::pair<double, double>> DEFAULT_GAINS;
extern std::unordered_map<std::string, std::pair<double, double>> JOINT_LIMITS;

void load_control_params(const std::string& path_in);

class ObsBuilder {
public:
    ObsBuilder();

    std::array<float, 45> build(
        const std::array<float, 3>& gyro,
        const std::array<float, 3>& gravity,
        const std::array<float, 3>& cmd_vel,
        const std::unordered_map<std::string, double>& joint_positions,
        const std::unordered_map<std::string, double>& joint_velocities
    ) const;

    void update_last_action(const std::array<float, 12>& action);

    static std::unordered_map<std::string, double> action_to_positions(const std::array<float, 12>& action);

private:
    std::array<float, 12> last_action_;
};

} // namespace biped_control_cpp