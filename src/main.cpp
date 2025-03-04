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

#include <camera.hpp>
#include <imgui_adaptors.hpp>
#include <mesh.h>
#include <sdl_adaptors.hpp>
#include <triangles_raytracing.hpp>

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
  Camera camera;
};
void pollEvents(ApplicationState &state);

int main(int, char **) {
  std::filesystem::path exec_path;
  {
    char path_cstr[PATH_MAX + 1] = {};
    std::ignore = readlink("/proc/self/exe", path_cstr, PATH_MAX);
    exec_path = path_cstr;
  }
  auto project_path = exec_path.parent_path().parent_path();
  auto resources = project_path / "resources";

  auto mesh_path = resources / "cube.obj";
  auto mesh = LoadMeshFromObj(mesh_path.c_str(), true);

  ApplicationState state;
  state.camera =
      Camera({0.0, 0.0f, 5.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});

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

    auto projMatrix = perspectiveMatrix(
        45.0f, static_cast<float>(state.W) / static_cast<float>(state.H), 0.01f,
        100.0f);
    auto projInv = inverse4x4(projMatrix);
    auto b = std::chrono::high_resolution_clock::now();
    state.image.clear(0);
    trace_triangles(state.image, mesh, state.camera, projInv);
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
      float3 cameraNewPos = state.camera.position();
      if (ImGui::InputFloat3("Camera Position", cameraNewPos.M)) {
        state.camera.resetPosition(cameraNewPos);
      }
      float3 up = state.camera.up();
      float3 right = state.camera.right();
      ImGui::Text("Camera Up (%0.3f, %0.3f, %0.3f)", up.x, up.y, up.z);
      ImGui::Text("Camera Right (%0.3f, %0.3f, %0.3f)", right.x, right.y, right.z);
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
  auto &io = ImGui::GetIO();
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
      state.image.resize(state.W, state.H);
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
      auto [dx, dy] = io.MouseDelta;
      state.camera.rotate(-dx, -dy);
    }
  }
}