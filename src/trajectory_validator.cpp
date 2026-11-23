#include "trajectory_validator.hpp"
#include <cmath>
#include <sstream>

TrajectoryValidator::TrajectoryValidator(std::vector<JointLimits> limits)
    : limits_(std::move(limits)) {}

bool TrajectoryValidator::validate(const std::vector<TrajectoryPoint>& points, std::string& error_msg) const {
    if (points.empty()) {
        error_msg = "Trajectory points cannot be empty";
        return false;
    }

    double last_time = -0.0001;
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& pt = points[i];
        if (pt.time_from_start_sec <= last_time) {
            error_msg = "Non-monotonic time stamp at point " + std::to_string(i);
            return false;
        }
        last_time = pt.time_from_start_sec;

        if (pt.positions.size() != limits_.size()) {
            error_msg = "Joint count mismatch at point " + std::to_string(i);
            return false;
        }

        for (size_t j = 0; j < limits_.size(); ++j) {
            if (pt.positions[j] < limits_[j].min_pos || pt.positions[j] > limits_[j].max_pos) {
                error_msg = "Joint position out of bounds for joint " + std::to_string(j) + " at point " + std::to_string(i);
                return false;
            }
            if (!pt.velocities.empty() && std::abs(pt.velocities[j]) > limits_[j].max_vel) {
                error_msg = "Velocity limit exceeded for joint " + std::to_string(j) + " at point " + std::to_string(i);
                return false;
            }
            if (!pt.accelerations.empty() && std::abs(pt.accelerations[j]) > limits_[j].max_acc) {
                error_msg = "Acceleration limit exceeded for joint " + std::to_string(j) + " at point " + std::to_string(i);
                return false;
            }
        }
    }

    return true;
}
