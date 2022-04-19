#include <SDL.h>
#include "sdl_wrapper.hpp"
#include "obstacle.hpp"
#include "draw.hpp"

int main(int argc, char * args[]) {
    // Screen Dims Constants
    const int SCREEN_WIDTH = 640;
    const int SCREEN_HEIGHT = 480;

    bool isquit = false;

    SDLWrapper sdl_obj(SCREEN_WIDTH, SCREEN_HEIGHT);

    // Contructing Tunnel and Accessing Shared Pointer
    auto obstacle_layer_object = Obstacle(SCREEN_WIDTH, SCREEN_HEIGHT);
    obstacle_layer_object.constructTunnel();
    auto obstacle_layer_shr_ptr = obstacle_layer_object.getObstacleLayer();

    // Const Draw Object and Drawing the tunnel
    auto draw_obj = Draw(SCREEN_WIDTH, SCREEN_HEIGHT);
    draw_obj.DrawPointsUsingLayer(obstacle_layer_shr_ptr, sdl_obj);

    sdl_obj.show();

    //Wait two seconds
    while (!isquit) {
        isquit = sdl_obj.check_poll_event();
    }

    return 0;
}
