#pragma once

#include <memory>
#include <mutex>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include <sdl_adaptors.hpp>

namespace imgui_adaptors {

using ContextHandler = std::shared_ptr<ImGuiContext>;
inline ContextHandler createContext(ImFontAtlas *shared_font_atlas = nullptr) {
  return {ImGui::CreateContext(shared_font_atlas), ImGui::DestroyContext};
}

class BackendManager {
public:
  BackendManager(const BackendManager &) = delete;
  BackendManager(BackendManager &&) = delete;
  BackendManager &operator=(const BackendManager &) = delete;
  BackendManager &operator=(BackendManager &&) = delete;

public:
  static BackendManager &getInstance() noexcept;
  void tryToInitialize(ContextHandler pContext,
                       sdl_adapters::WindowHandler pWindow,
                       sdl_adapters::RendererHandler pRenderer);

private:
  BackendManager() = default;
  bool m_isInitialized = false;
  std::mutex m_FlagLock;
  ContextHandler m_pContext;
  sdl_adapters::WindowHandler m_pWindow;
  sdl_adapters::RendererHandler m_pRenderer;

public:
  ~BackendManager();
};

struct WindowGuard {
  WindowGuard(const char *name, bool *p_open = nullptr,
              ImGuiWindowFlags flags = 0) noexcept;
  ~WindowGuard();
};

} // namespace imgui_adaptors