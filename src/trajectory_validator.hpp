#pragma once

#include <vector>
#include <string>

struct JointLimits {
    double min_pos;
    double max_pos;
    double max_vel;
    double max_acc;
};

struct TrajectoryPoint {
    double time_from_start_sec;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
};

class TrajectoryValidator {
public:
    explicit TrajectoryValidator(std::vector<JointLimits> limits);

    bool validate(const std::vector<TrajectoryPoint>& points, std::string& error_msg) const;
    size_t get_num_joints() const { return limits_.size(); }

private:
    std::vector<JointLimits> limits_;
};
