struct Position {
    Position(float x, float y) : x_(x), y_(y) {}
    float x_, y_;
};

class Robot {
public:
    Robot(int x, int y) : pos_({static_cast<float>(x), static_cast<float>(y)}) {}
    void update_pos(float x, float y) {
        pos_.x_ = x;
        pos_.y_ = y;
    }

    void get_pos(float &x, float &y) {
        x = this->pos_.x_;
        y = this->pos_.y_;
    }

private:
    Position pos_;
};
