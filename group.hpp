#ifndef GROUP_HPP
#define GROUP_HPP

#include <vector>
#include <utility>
#include <cmath>
#include <random>

class Group {
public:
    Group(double x, double y, int num_of_robots, int radius)
    {
        center_x = x;
        center_y = y;
        this->radius = static_cast<double>(radius);

        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> distr(-radius, radius); // define the range

        int current_count = 0;
        while(current_count < num_of_robots)
        {
            double x = distr(gen);
            double y = distr(gen);

            if(std::hypot(x, y) < radius)
            {
                robots.emplace_back(x+center_x, y+center_y);
                current_count++;
            }
        }
    }

    std::vector<std::pair<double, double> > getRobotVector()
    {
        return this->robots;
    }

    std::tuple<double, double, double> getCircleInfo()
    {
        return std::make_tuple(center_x, center_y, radius);
    }

    void updateRobotPosition(std::vector<std::pair<double, double> > &update_vector)
    {
        double ce_x = 0;
        double ce_y = 0;
        for(int i=0; i < update_vector.size(); i++)
        {
            robots[i].first += update_vector[i].first;
            robots[i].second += update_vector[i].second;
            ce_x += robots[i].first;
            ce_y += robots[i].second;
        }

        // Updating center value
        this->center_x = ce_x/getNumberOfRobots();
        this->center_y = ce_y/getNumberOfRobots();

    }

    void updateCenter()
    {
        int max_index = 0;
        double max_distance_x = robots[max_index].first-center_x;
        double max_distance_y = robots[max_index].second-center_y;
        double max_radius = std::hypot(max_distance_x, max_distance_y);

        for(int i=1; i<robots.size(); i++)
        {
            max_distance_x = robots[i].first-center_x;
            max_distance_y = robots[i].second-center_y;
            if(std::hypot(max_distance_x, max_distance_y) > max_radius)
            {
                max_radius = std::hypot(max_distance_x, max_distance_y);
            }
        }

        radius = max_radius;
    }

    void updateCenterManualOverride(double rad)
    {
        radius = rad;
    }

    int getNumberOfRobots()
    {
        return robots.size();
    }

private:
    std::vector<std::pair<double, double> > robots;
    double radius;
    double center_x, center_y;
};

#endif // GROUP_HPP