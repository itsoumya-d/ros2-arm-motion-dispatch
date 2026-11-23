#include "trajectory_validator.hpp"
#include <cassert>
#include <iostream>

void test_valid_trajectory() {
    std::vector<JointLimits> limits = {
        {-3.14, 3.14, 1.5, 3.0},
        {-1.57, 1.57, 1.0, 2.0}
    };
    TrajectoryValidator validator(limits);

    std::vector<TrajectoryPoint> points = {
        {0.0, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}},
        {1.0, {0.5, 0.2}, {0.5, 0.2}, {0.1, 0.1}}
    };

    std::string err;
    bool ok = validator.validate(points, err);
    assert(ok);
    assert(err.empty());
    std::cout << "test_valid_trajectory passed\n";
}

void test_out_of_bounds() {
    std::vector<JointLimits> limits = {
        {-1.0, 1.0, 1.0, 1.0}
    };
    TrajectoryValidator validator(limits);

    std::vector<TrajectoryPoint> points = {
        {0.0, {1.5}, {0.0}, {0.0}}
    };

    std::string err;
    bool ok = validator.validate(points, err);
    assert(!ok);
    assert(!err.empty());
    std::cout << "test_out_of_bounds passed\n";
}

int main() {
    test_valid_trajectory();
    test_out_of_bounds();
    std::cout << "All C++ baseline tests passed!\n";
    return 0;
}
