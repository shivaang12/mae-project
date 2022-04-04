#include <SDL.h>

class SDLWrapper {
public:
    SDLWrapper(int width, int height) : height_(height), width_(width)  {
        SDL_Init(SDL_INIT_VIDEO);
        SDL_CreateWindowAndRenderer(width_, height_, 0, &window_, &renderer_);
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 0);
        SDL_RenderClear(renderer_);
        SDL_RenderPresent(renderer_);
    }
    ~SDLWrapper() {
        SDL_DestroyWindow( this->window_ );
        SDL_Quit();
    }

    bool check_poll_event() {
        if (SDL_PollEvent( & this->event_)) {
            if (event_.type == SDL_QUIT) {
                return true;
            }
        }
        
        return false;
    }

    void update() {
        SDL_RenderClear(this->renderer_);
    }

    void show() {
        SDL_RenderPresent(this->renderer_);
    }

    void update_and_show() {
        this->update();
        this->show();
    }

    void clear() {
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 0);
        this->update_and_show();
    }

private:
    // SDL related Variables
    SDL_Window* window_;
    SDL_Renderer *renderer_;
    SDL_Event event_;

    // Non SDL related Variables
    int width_;
    int height_;
};