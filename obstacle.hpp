#include <vector>
#include <memory>

class Obstacle {
public:
    Obstacle(int width, int height) : width_(width), height_(height) {
        obstacle_layer_ = std::make_shared<std::vector<int>>(std::vector<int>(height*width, 0));
    }

    void constructTunnel() const {
        for (int i=310; i < 330; i++) {
            for (int j=0; j <220; j++) {
                (*obstacle_layer_)[j*this->width_ + i] = 1;
            }

            for (int j=260; j <480; j++) {
                (*obstacle_layer_)[j*this->width_ + i] = 1;
            }
        }
    }

    std::shared_ptr<std::vector<int>> getObstacleLayer() {
        return obstacle_layer_;
    }

private:
    int width_, height_;
    std::shared_ptr<std::vector<int>> obstacle_layer_;
};
