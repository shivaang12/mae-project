#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <vector>
#include <utility>
#include <tuple>
#include <cmath>
#include <algorithm>
#include "robot.hpp"

class Controller {
public:
    Controller() = default;
    void register_path(std::vector< std::tuple<int, int, int> > &path)
    {
        path_ = path;
    }

    std::pair<int, int> getUpdateVector(Robot robot)
    {
        double current_x, current_y;
        robot.get_pos(current_x, current_y);
        int min_index = find_closest_index(current_x, current_y);

        int target_index = min_index + 1;

        // Checking if the index is goal point
        if (min_index == (path_.size()-1))
        {
            // If Goal point then check how far it is
            double goal_x = std::get<0>(path_[min_index]);
            double goal_y = std::get<1>(path_[min_index]);
            if (calEuclideanDistance(current_x - goal_x, current_y - goal_y) < 2)
            {
                // Goal has reached
                return;
            } else {
                // If not
                // Make the Goal Point the Target index
                target_index = min_index;
            }
        }

        // If not the goal point then find the dx and dy for {min_index + 1}th index
        double target_x = std::get<0>(path_[target_index]);
        double target_y = std::get<1>(path_[target_index]);
        double dx = target_x-current_x;
        double dy = target_y-current_y;

        // Now limit -1 <= {dx, dy} <= 1
        if ((dx < 0) && (dx < -1))
        {
            dx = -1;
        }

        if ((dx > 0) && (dx > 1))
        {
            dx = 1;
        }

        if ((dy < 0) && (dy < -1))
        {
            dy = -1;
        }

        if ((dy > 0) && (dy > 1))
        {
            dy = 1;
        }
    }

    int find_closest_index(double x, double y)
    {
        int index = -1;
        std::vector<int> vector_distance;
        vector_distance.reserve(path_.size());

        for (auto &item : path_)
        {
            vector_distance.emplace_back(calEuclideanDistance(x-std::get<0>(item), y-std::get<1>(item)));
        }

        // Now find min index
        int min_index = std::min_element(vector_distance.begin(), vector_distance.end()) - vector_distance.begin();

        return min_index;
    }

    double calEuclideanDistance(double x, double y)
    {
        return std::hypot(x, y);
    }

private:
    std::vector<std::tuple<int, int, int> > path_;
};

#endif // CONTROLLER_HPP