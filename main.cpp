#include <SDL.h>
#include "sdl_wrapper.hpp"

int main(int argc, char * args[]) {
    // Screen Dims Constants
    const int SCREEN_WIDTH = 640;
    const int SCREEN_HEIGHT = 480;

    bool isquit = false;

    SDLWrapper sdl_obj(SCREEN_WIDTH, SCREEN_HEIGHT);

    //Wait two seconds
    while (!isquit) {
        isquit = sdl_obj.check_poll_event();
    }

    return 0;
}
