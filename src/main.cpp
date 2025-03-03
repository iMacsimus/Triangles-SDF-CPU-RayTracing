#ifndef linux
static_assert(false, "This code is valid for Ubuntu x64 linux");
#endif

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <LiteMath/Image2d.h>
#include <LiteMath/LiteMath.h>
#include <SDL.h>
#include <SDL_keycode.h>

#include <imgui_adaptors.hpp>
#include <mesh.h>
#include <quaternion.hpp>
#include <sdl_adaptors.hpp>

using namespace cmesh4;
using namespace LiteMath;
using namespace LiteImage;

struct ApplicationState {
  bool shouldBeClosed = false;
  sdl_adapters::WindowHandler pWindow;
  sdl_adapters::RendererHandler pRenderer;
  sdl_adapters::TextureHandler pSDLTexture;
  Image2D<uint32_t> image;
  int W = 1280, H = 720;
};
void pollEvents(ApplicationState &state);

int main(int, char **) {
  ApplicationState state;

  auto &sdlManager = sdl_adapters::SDLManager::getInstance();
  sdlManager.tryToInitialize(SDL_INIT_VIDEO | SDL_INIT_TIMER);

  // Create window with SDL_Renderer graphics context
  state.pWindow = sdl_adapters::createWindow(
      "Triangles/SDL Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      state.W, state.H, SDL_WINDOW_RESIZABLE);
  state.pRenderer = sdl_adapters::createRenderer(
      state.pWindow, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  state.pSDLTexture = sdl_adapters::createTexture(
      state.pRenderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING,
      state.W, state.H);
  state.image.resize(state.W, state.H);

  // Setup Dear ImGui context
  auto pImGuiContext = imgui_adaptors::createContext();
  ImGui::SetCurrentContext(pImGuiContext.get());
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

  // Setup Platform/Renderer backends
  auto &imguiBackend = imgui_adaptors::BackendManager::getInstance();
  imguiBackend.tryToInitialize(pImGuiContext, state.pWindow, state.pRenderer);

  while (!state.shouldBeClosed) {
    // Poll and handle events (inputs, window resize, etc.)
    pollEvents(state);

    if (SDL_GetWindowFlags(state.pWindow.get()) & SDL_WINDOW_MINIMIZED) {
      SDL_Delay(10);
      continue;
    }

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    auto b = std::chrono::high_resolution_clock::now();
    for (int indx = 0; indx < state.W * state.H; ++indx) {
      uchar4 color = {static_cast<uint8_t>(rand() % 255),
                      static_cast<uint8_t>(rand() % 255),
                      static_cast<uint8_t>(rand() % 255), 255};
      int2 xy = {indx % state.W, indx / state.W};
      state.image[xy] = color.u32;
    }
    auto e = std::chrono::high_resolution_clock::now();
    float time =
        static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(e - b)
                .count()) /
        1e3f;
    // Window "Properties"
    {
      // starts the window, ends when out of the scope
      imgui_adaptors::WindowGuard wg("Properties");
      ImGui::Text("Window Resolution: %dx%d", state.W, state.H);
      ImGui::Text("Render Time: %.03fms", time);
    }

    // Rendering
    SDL_RenderClear(state.pRenderer.get());
    SDL_UpdateTexture(state.pSDLTexture.get(), nullptr, state.image.data(),
                      state.W * sizeof(uint32_t));
    SDL_RenderCopy(state.pRenderer.get(), state.pSDLTexture.get(), nullptr,
                   nullptr);

    ImGui::Render();
    SDL_RenderSetScale(state.pRenderer.get(), io.DisplayFramebufferScale.x,
                       io.DisplayFramebufferScale.y);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(),
                                          state.pRenderer.get());
    SDL_RenderPresent(state.pRenderer.get());
  }
  return 0;
}

void pollEvents(ApplicationState &state) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    if (event.type == SDL_QUIT)
      state.shouldBeClosed = true;
    if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == SDL_GetWindowID(state.pWindow.get()))
      state.shouldBeClosed = true;
    if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_RESIZED &&
        event.window.windowID == SDL_GetWindowID(state.pWindow.get())) {
      SDL_GetWindowSize(state.pWindow.get(), &state.W, &state.H);
      state.pSDLTexture = sdl_adapters::createTexture(
          state.pRenderer, SDL_PIXELFORMAT_ABGR8888,
          SDL_TEXTUREACCESS_STREAMING, state.W, state.H);
    }
  }
}