#include <SDL.h>
#include <memory>
#include <vector>
#include "sdl_wrapper.hpp"

class Draw {
public:
    Draw(int width, int height):width_(width), height_(height) {

    }

    void DrawPointsUsingLayer(std::shared_ptr<std::vector<int>> obstacle_layer_ptr, SDLWrapper& wrapper) {
        // Set Color
        wrapper.setColor(255, 255, 0);
        // Draw Points
        for(int i=0; i < obstacle_layer_ptr->size(); i++){
            if ((*obstacle_layer_ptr)[i] == 1) {
                int x = i%width_;
                int y = i/width_;
                wrapper.drawPoint(x, y);
            }
        }
    }

private:
    int width_, height_;
};
