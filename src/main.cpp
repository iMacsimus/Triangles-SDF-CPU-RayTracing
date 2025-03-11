#ifndef linux
static_assert(false, "This code is valid for Ubuntu x64 linux");
#endif

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <string>
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
using namespace std::string_literals;

struct ApplicationState {
  bool shouldBeClosed = false;
  bool meshLoaded = false;
  bool gridBuilt = false;
  bool octreeBuilt = false;
  sdl_adapters::WindowHandler pWindow;
  sdl_adapters::RendererHandler pRenderer;
  sdl_adapters::TextureHandler pSDLTexture;
  FrameBuffer frameBuf;
  int W = 1280, H = 720;
  Camera camera;
};
void pollEvents(ApplicationState &state);
cmesh4::SimpleMesh loadAndScale(std::filesystem::path path);

int main(int, char **) {
  ApplicationState state;
  std::filesystem::path exec_path;
  {
    char path_cstr[PATH_MAX + 1] = {};
    std::ignore = readlink("/proc/self/exe", path_cstr, PATH_MAX);
    exec_path = path_cstr;
  }
  auto project_path = exec_path.parent_path().parent_path();
  auto resources = project_path / "resources";
  auto mesh_path = resources / "cube.obj";

  Renderer renderer;
  renderer.lightPos = {2, 2, 2};
  auto pBVHScene = std::make_shared<BVHBuilder>();
  auto pGroundPlane =
      std::make_shared<Plane>(LiteMath::float3{0.0f, 1.0f, 0.0f}, -1.0f);
  SceneUnion fullScene(pBVHScene, pGroundPlane);
  bool enableGroundPlane = true;
  cmesh4::SimpleMesh mesh;
  std::future<void> asyncResult;
  bool needToLoadMesh = false;
  int dotsCount = 3;

  int currentShadingMode = 1;
  ShadingMode shadingModes[3] = {ShadingMode::Color, ShadingMode::Lambert,
                                 ShadingMode::Normal};
  const char *shadinModesStr[3] = {"Color", "Lambert", "Normal"};

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
  state.frameBuf.resize(state.W, state.H);
  state.camera =
      Camera({0.0, 0.0f, 2.5f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
  state.camera.setLockUp(true);

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

    if (needToLoadMesh) {
      ImVec2 windowSize = {300.0f, 100.0f};
      ImGui::SetNextWindowSize(windowSize);
      ImGui::SetNextWindowPos(
          ImVec2{(static_cast<float>(state.W) - windowSize[0]) / 2.0f,
                 (static_cast<float>(state.H) - windowSize[1]) / 2.0f});
      imgui_adaptors::WindowGuard wg("Mesh Loading", nullptr,
                                     ImGuiWindowFlags_NoCollapse |
                                         ImGuiWindowFlags_NoResize);
      std::string dots(dotsCount, '.');
      ImGui::Text("Reading file and constructing BVH.");
      ImGui::Text("Please, wait%s", dots.c_str());
      dotsCount = (dotsCount % 3) + 1;
      if (asyncResult.wait_for(std::chrono::milliseconds(30)) ==
          std::future_status::ready) {
        needToLoadMesh = false;
        state.meshLoaded = true;
        state.camera = Camera({0.0f, 0.0f, 2.5f}, {0.0f, 0.0f, 0.0f});
        state.camera.setLockUp(true);
      }
    }

    // Window "Properties"
    {
      ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
      ImGui::SetNextWindowSize(ImVec2(350.0f, static_cast<float>(state.H)));
      imgui_adaptors::WindowGuard wg("Preferences", nullptr,
                                     ImGuiWindowFlags_NoResize |
                                         ImGuiWindowFlags_NoMove);
      ImGui::Text("Mesh Settings:");
      if (ImGui::Button("Load mesh")) {
        needToLoadMesh = true;
        state.meshLoaded = false;
        std::string command =
            "zenity --file-selection --title=\"Select model\" --filename=\""s +
            mesh_path.c_str() +
            "\" --file-filter=\"OBJ Files | *.obj\" --file-filter=\"Grid Files "
            "| *.grid\" --file-filter=\"Octree Files | *.octree\"";
        FILE *pipe = popen(command.c_str(), "r");
        char buffer[PATH_MAX + 1] = {};
        std::string result = "";
        while (fgets(buffer, sizeof(buffer) - 1, pipe) != nullptr) {
          result += buffer;
          std::fill(buffer, buffer + sizeof(buffer), 0);
        }
        result.pop_back();
        mesh_path = result;
        pclose(pipe);

        asyncResult = std::async(std::launch::async, [&]() {
          mesh = loadAndScale(mesh_path);
          auto newBox = calc_bbox(mesh);
          *pGroundPlane = Plane(float3{0.0f, 1.0f, 0.0f}, newBox.boxMin.y);
          pBVHScene->perform(std::move(mesh));
        });
      }

      float time = 0.0f;

      if (state.meshLoaded) {
        state.frameBuf.clear();
        auto proj = perspectiveMatrix(
            45.0f, static_cast<float>(state.W) / static_cast<float>(state.H),
            0.01f, 100.0f);
        auto projInv = inverse4x4(proj);
        if (!enableGroundPlane) {
          time =
              renderer.draw(*pBVHScene, state.frameBuf, state.camera, projInv);
        } else {
          time =
              renderer.draw(fullScene, state.frameBuf, state.camera, projInv);
        }
      }

      ImGui::Text("Camera Settings:");
      float3 cameraNewPos = state.camera.position();
      if (ImGui::InputFloat3("Camera Position", cameraNewPos.M)) {
        state.camera.resetPosition(cameraNewPos);
      }
      bool isLockedUp = state.camera.isLockedUp();
      ImGui::Checkbox("Lock vector Up", &isLockedUp);
      state.camera.setLockUp(isLockedUp);

      ImGui::Text("Renderer Settings:");
      ImGui::Checkbox("Ground Plane", &enableGroundPlane);
      ImGui::ListBox("Shading Mode", &currentShadingMode, shadinModesStr, 3);
      renderer.shadingMode = shadingModes[currentShadingMode];
      if (renderer.shadingMode == ShadingMode::Lambert) {
        ImGui::Checkbox("Enable shadows", &renderer.enableShadows);
        ImGui::Checkbox("Enable reflections", &renderer.enableReflections);
      }
      float3 up = state.camera.up();
      float3 right = state.camera.right();
      ImGui::Text("Debug Info:");
      ImGui::Text("\tCamera Up (%0.3f, %0.3f, %0.3f)", up.x, up.y, up.z);
      ImGui::Text("\tCamera Right (%0.3f, %0.3f, %0.3f)", right.x, right.y,
                  right.z);
      ImGui::Text("\tWindow Resolution: %dx%d", state.W, state.H);
      ImGui::Text("\tRender Time: %.03fms", time);
      if (state.meshLoaded) {
        ImGui::Text(
            "\tBVH memory usage: %f MiB",
            static_cast<float>(pBVHScene->nodesCount() * sizeof(BVH8Node)) /
                static_cast<float>(2 << 20));
      }
    }

    // Rendering
    SDL_RenderClear(state.pRenderer.get());
    SDL_UpdateTexture(state.pSDLTexture.get(), nullptr,
                      state.frameBuf.color.data(), state.W * sizeof(uint32_t));
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
  static float shift = 1.0f;
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
      state.frameBuf.resize(state.W, state.H);
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
      auto [dx, dy] = io.MouseDelta;
      state.camera.rotate(-dx, -dy);
    }
    if (event.type == SDL_EventType::SDL_MOUSEWHEEL &&
        event.window.windowID == SDL_GetWindowID(state.pWindow.get()) &&
        !io.WantCaptureMouse) {
      float distance = length(state.camera.target() - state.camera.position());
      float r = static_cast<float>(event.wheel.y) * distance / 25.0f;
      state.camera.resetPosition(state.camera.position() +
                                 state.camera.forward() * r);
    }
    if (event.type == SDL_EventType::SDL_KEYDOWN &&
        event.window.windowID == SDL_GetWindowID(state.pWindow.get()) &&
        !io.WantCaptureMouse) {
      float distance = length(state.camera.target() - state.camera.position());
      switch (event.key.keysym.sym) {
      case SDLK_KP_1:
        state.camera =
            Camera({0, 0, state.camera.target().z + distance * shift},
                   state.camera.target());
        state.camera.setLockUp(true);
        break;
      case SDLK_KP_3:
        state.camera =
            Camera({state.camera.target().x + distance * shift, 0, 0},
                   state.camera.target());
        state.camera.setLockUp(true);
        break;
      case SDLK_KP_7:
        state.camera =
            Camera({0, state.camera.target().y + distance * shift, 0},
                   state.camera.target(), {0, 0, -1 * shift});
        state.camera.setLockUp(true);
        break;
      case SDLK_LSHIFT:
        shift = -1.0f;
        state.camera.setLockUp(true);
        break;
      }
    }
    if (event.type == SDL_EventType::SDL_KEYUP &&
        event.window.windowID == SDL_GetWindowID(state.pWindow.get()) &&
        !io.WantCaptureMouse) {
      if (event.key.keysym.sym == SDLK_LSHIFT) {
        shift = 1.0f;
      }
    }
  }
}

cmesh4::SimpleMesh loadAndScale(std::filesystem::path path) {
  auto mesh = LoadMeshFromObj(path.c_str(), true);

  auto bbox = calc_bbox(mesh);
  auto center = (bbox.boxMin + bbox.boxMax) / 2.0f;
  auto scale = length(bbox.boxMax - center);
  for (auto &v : mesh.vPos4f) {
    auto w = v.w;
    v /= w;
    float3 scaled = to_float3(v);
    scaled -= center;
    scaled /= scale;
    v = to_float4(scaled, 1.0f);
    v *= w;
  }

  return mesh;
}