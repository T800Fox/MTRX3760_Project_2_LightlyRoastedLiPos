#include <SDL.h>
#include <cmath>
#include <string>

#include "mtrx3760_lrl_robot_vis/render_objs.hpp"
#include "mtrx3760_lrl_robot_vis/socket.hpp"



// Main
int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 600;

    SDL_Window* window = SDL_CreateWindow("SDL2 2D Camera Example",
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    Cam2D cam(Point(0, 0), 1.0f, WINDOW_WIDTH, WINDOW_HEIGHT, 1.0f, 0.2f);

    Grid grid(Point(0,0), 50.0f, &cam, renderer);

    // Create objects
    Line line(Point(-1, -1), Point(2, 2), &cam, renderer, true);
    Circle circle(Point(1, 1), 30, &cam, renderer, true);
    Rect rect(Point(0, 0), 10.0f, Point(100, 50), &cam, renderer, true);

    bool quit = false;
    SDL_Event e;

    int scroll_event = 0;
    bool mouse_pressed = false;
    Point mouse_pos;

    while (!quit) {
        scroll_event = 0;

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
            else if (e.type == SDL_MOUSEWHEEL) {
                scroll_event = e.wheel.y;
            }
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    mouse_pressed = true;
                }
            }
            else if (e.type == SDL_MOUSEBUTTONUP) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    mouse_pressed = false;
                }
            }
        }

        int mx, my;
        SDL_GetMouseState(&mx, &my);
        mouse_pos = Point((float)mx, (float)my);

        
       /* rect.set_pos(shared_data.robot_pos);
        rect.set_angle(shared_data.robot_rot);
        std::cout << shared_data.robot_rot << '\n';*/


        cam.update(mouse_pos, mouse_pressed, scroll_event);
        

        //Rendering:
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);

        grid.render();
        line.render();
        circle.render();
        rect.render();


        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
