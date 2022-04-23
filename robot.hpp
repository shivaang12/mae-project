#include <utility>
#include <iostream>

struct Position {
    Position(double x, double y) : x_(x), y_(y) {}
    double x_, y_;
};

class Robot {
public:
    Robot(int x, int y) : pos_({static_cast<double>(x), static_cast<double>(y)}) {}
    void update_pos(double x, double y) {
        pos_.x_ = x;
        pos_.y_ = y;
    }

    void get_pos(double &x, double &y) {
        x = this->pos_.x_;
        y = this->pos_.y_;
    }

    void change_pose_by(double dx, double dy) {
        this->pos_.x_ += dx;
        this->pos_.y_ += dy;
    }

    std::pair<double, double> getPoseByPair()
    {
        return std::make_pair(this->pos_.x_, this->pos_.y_);
    }

    void print_robot_path()
    {
        std::cout << this->pos_.x_ << " " << this->pos_.y_ << '\n';
    }

private:
    Position pos_;
};
