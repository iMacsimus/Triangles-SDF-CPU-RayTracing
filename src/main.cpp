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
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;

struct ApplicationState {
  bool shouldBeClosed = false;
  sdl_adapters::WindowHandler pWindow;
};
void pollEvents(ApplicationState &state);

int main(int, char **) {
  ApplicationState state;

  int W = 1280, H = 720;
  auto &sdlManager = sdl_adapters::SDLManager::getInstance();
  sdlManager.tryToInitialize(SDL_INIT_VIDEO | SDL_INIT_TIMER);

  // Create window with SDL_Renderer graphics context
  auto pWindow = sdl_adapters::createWindow(
      "Triangles/SDL Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W,
      H, SDL_WINDOW_RESIZABLE);
  auto pRenderer = sdl_adapters::createRenderer(
      pWindow, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  state.pWindow = pWindow;

  // Setup Dear ImGui context
  auto pImGuiContext = imgui_adaptors::createContext();
  ImGui::SetCurrentContext(pImGuiContext.get());
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

  // Setup Platform/Renderer backends
  auto &imguiBackend = imgui_adaptors::BackendManager::getInstance();
  imguiBackend.tryToInitialize(pImGuiContext, pWindow, pRenderer);

  // Our state
  ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

  while (!state.shouldBeClosed) {
    // Poll and handle events (inputs, window resize, etc.)
    pollEvents(state);

    if (SDL_GetWindowFlags(pWindow.get()) & SDL_WINDOW_MINIMIZED) {
      SDL_Delay(10);
      continue;
    }

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Window "Properties"
    {
      // starts the window, ends when out of the scope
      imgui_adaptors::WindowGuard wg("Properties");

      ImGui::Text("This is some useful text.");
    }

    // Rendering
    ImGui::Render();
    SDL_RenderSetScale(pRenderer.get(), io.DisplayFramebufferScale.x,
                       io.DisplayFramebufferScale.y);
    SDL_SetRenderDrawColor(pRenderer.get(), (Uint8)(clear_color.x * 255),
                           (Uint8)(clear_color.y * 255),
                           (Uint8)(clear_color.z * 255),
                           (Uint8)(clear_color.w * 255));
    SDL_RenderClear(pRenderer.get());
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(),
                                          pRenderer.get());
    SDL_RenderPresent(pRenderer.get());
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
  }
}