#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <vector>
#include <utility>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "group.hpp"

class Controller {
public:
    Controller(){
        last_index = -1;
    };
    void register_path(std::vector< std::tuple<int, int, int> > &path)
    {
        path_ = path;
    }

    bool checkGoal(Group &robot)
    {
        double current_x, current_y, current_r;
        auto info = robot.getCircleInfo();
        current_x = std::get<0>(info);
        current_y = std::get<1>(info);
        current_r = std::get<2>(info);
        int min_index = find_closest_index(current_x, current_y);
        
        if (min_index == last_index)
        {

        }

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
                return true;
            } else {
                // If not
                // Make the Goal Point the Target index
                return false;
            }
        } else {
            return false;
        }
    }

    std::vector<std::pair<double, double> > getRobotPose(Group &robot)
    {
        double current_x, current_y, current_r;
        auto info = robot.getCircleInfo();
        current_x = std::get<0>(info);
        current_y = std::get<1>(info);
        current_r = std::get<2>(info);
        int min_index = find_closest_index(current_x, current_y);

        std::cout << last_index << " " << min_index << '\n';
        std::cout << current_x << " " << current_y << '\n';

        int target_index = min_index + 1;

        if ((min_index == last_index) && (target_index + 1 < path_.size()))
        {
            target_index++;
        }

        // Checking if the index is goal point
        if (min_index == (path_.size()-1))
        {
            // If Goal point then check how far it is
            double goal_x = std::get<0>(path_[min_index]);
            double goal_y = std::get<1>(path_[min_index]);
            if (calEuclideanDistance(current_x - goal_x, current_y - goal_y) < 2)
            {
                // Goal has reached
                std::vector<std::pair<double, double> > return_vector(robot.getNumberOfRobots(), std::make_pair(0.0, 0.0));
                return return_vector;
            } else {
                // If not
                // Make the Goal Point the Target index
                target_index = min_index;
            }
        }

        // If not the goal point then find the dx and dy for {min_index + 1}th index
        double target_x = std::get<0>(path_[target_index]);
        double target_y = std::get<1>(path_[target_index]);
        double target_r = std::get<2>(path_[target_index]);

        double dx = target_x-current_x;
        double dy = target_y-current_y;
        double dr = target_r-current_r;
        std::cout << dr << '\n';

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

        if ((dr < 0) && (dr < -1))
        {
            dr = -1;
        }

        if ((dr > 0) && (dr > 1))
        {
            dr = 1;
        }

        // We only need update vector for Scale Change because Position update vector will be
        //  same as the centroid update vector for all the robots
        std::vector<std::pair<double, double> > norm_vector;
        norm_vector.reserve(robot.getNumberOfRobots());

        if(std::abs(dr) < 0.0001)
        {
            //aking the norm
            double norm_x = dx/hypot(dx, dy);
            double norm_y = dy/hypot(dx, dy);
            std::vector<std::pair<double, double> > temp_vector(robot.getNumberOfRobots(), std::make_pair(norm_x, norm_y));
            norm_vector = temp_vector;
        } else {
            std::vector<std::pair<double, double> > scale_update_vector = getScaleUpdateVector(robot, dr);
            for (const auto &item : scale_update_vector)
            {
                // 1. Adding the vector
                double tx = dx + item.first;
                double ty = dy + item.second;

                // 2. Taking the norm
                double norm_x = tx/hypot(tx, ty);
                double norm_y = ty/hypot(tx, ty);
                
                norm_vector.emplace_back(norm_x, norm_y);
            }

        }

        last_index = target_index;

        return norm_vector;

    }

    std::vector<std::pair<double, double> > getScaleUpdateVector(Group &robots, double dr)
    {

        // First Check if the max radius is less than what we want

        auto info = robots.getCircleInfo();
        auto robot_vector = robots.getRobotVector();

        int max_index = 1;

        for(int i=1; i<robot_vector.size(); i++)
        {
            if(std::hypot(robot_vector[max_index].first-std::get<0>(info), robot_vector[max_index].second-std::get<1>(info)) < )
        }

        std::vector<std::pair<double, double> > returnVector;
        returnVector.reserve(robot_vector.size());

        for (const auto &item : robot_vector)
        {
            double x, y;
            x = item.first - std::get<0>(info);
            y = item.second - std::get<1>(info);
            double dx = ((std::get<2>(info)-dr)/std::get<2>(info)) * x;
            double dy = ((std::get<2>(info)-dr)/std::get<2>(info)) * y;
            returnVector.emplace_back(dx, dy);
        }

        return returnVector;
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
    int last_index;
};

#endif // CONTROLLER_HPP