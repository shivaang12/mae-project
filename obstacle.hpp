#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <vector>
#include <memory>
#include <cmath>

class Obstacle {
public:
    Obstacle(int width, int height) : width_(width), height_(height) {
        obstacle_layer_ = std::make_shared<std::vector<int>>(std::vector<int>(height*width, 0));
        obstacle_points_.reserve((330 - 310) * (480 - 260));
    }

    void constructTunnel() {
        for (int i=310; i < 330; i++) {
            for (int j=0; j <220; j++) {
                (*obstacle_layer_)[j*this->width_ + i] = 1;
                obstacle_points_.emplace_back(i, j);
            }

            for (int j=260; j <480; j++) {
                (*obstacle_layer_)[j*this->width_ + i] = 1;
                obstacle_points_.emplace_back(i, j);
            }
        }
    }

    std::shared_ptr<std::vector<int>> getObstacleLayer() const {
        return obstacle_layer_;
    }

    std::shared_ptr<std::vector<std::pair<int, int>>> getObstacleList() const {
        return std::make_shared<std::vector<std::pair<int, int>>>(obstacle_points_);
    }

    std::vector<std::pair<int, int>> getCirclePoints (int x, int y, int z) const {
        std::vector<std::pair<int, int>> return_vector;
       
        for (int i=x-z; i<x+z; i++)
        {
            for(int j=y-z; j<y+z; j++)
            {
                if((std::hypot(i-x, j-y) < z) && (std::hypot(i-x, j-y) > (z-1)))
                {
                    return_vector.emplace_back(i, j);
                }
            }
        }

        return return_vector;
    }

private:
    int width_, height_;
    std::shared_ptr<std::vector<int>> obstacle_layer_;
    std::vector<std::pair<int, int>> obstacle_points_;
};

class LiveObstacle {
public:
    LiveObstacle(int x_size, int y_size):x_size(x_size), y_size(y_size) {}

    void configure(double start_x, double start_y, double dx, double dy)
    {
        this->start_x = start_x;
        this->start_y = start_y;
        this->dx = dx;
        this->dy = dy;
    }

    std::vector<std::pair<int, int>> getObtaclePoints()
    {
        std::vector<std::pair<int, int>> return_vector;

        for(int i=start_x-x_size; i<start_x+x_size; i++)
        {
            for(int j=start_y-y_size; j<start_y+y_size; j++)
            {
                return_vector.emplace_back(i, j);
            }
        }

        return return_vector;
    }

    void next()
    {
        this->start_x -= this->dx;
        this->start_y -= this->dy;
    }



private:
    int x_size, y_size;
    double start_x, start_y, dx, dy;
};
#endif // OBSTACLE_HPP
