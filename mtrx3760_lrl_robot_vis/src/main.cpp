#include <SDL.h>
#include <SDL_ttf.h>
#include <cmath>
#include <string>
#include "render_objs.hpp"
#include "socket.hpp"
#include "visualiser.hpp"




// Main
int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    TTF_Init(); //Init text renderer


    const int WINDOW_WIDTH = 1280;
    const int WINDOW_HEIGHT = 720;

    SDL_Window* window = SDL_CreateWindow("Warehouse Robot Interface",
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


    Uint32 lastTime = SDL_GetTicks();
    int frames = 0;


    Visualiser visualiser(window, renderer);


    bool quit = false;
    SDL_Event e;

    int scroll_event = 0;
    bool mouse_pressed = false;
    Point mouse_pos;

    while (!quit) {
        scroll_event = 0;
        Uint64 start = SDL_GetPerformanceCounter();

        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e); //pass event to IMGUI to proccess

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
        mouse_pos = Point{(float)mx, (float)my};

        //Update!!
        visualiser.update(mouse_pos, mouse_pressed, scroll_event);
        
    

        //Rendering:
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);

        visualiser.render();

        SDL_RenderPresent(renderer);


        Uint64 end = SDL_GetPerformanceCounter();
        double delta = (end - start) / (double)SDL_GetPerformanceFrequency();
        std::cout << "Frame time: " << delta*1000.0 << " ms, FPS: " << 1.0/delta << "\r";

    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
