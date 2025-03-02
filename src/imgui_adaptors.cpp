#include "imgui_adaptors.hpp"

namespace imgui_adaptors {

BackendManager &BackendManager::getInstance() noexcept {
  static BackendManager instance;
  return instance;
}

void BackendManager::tryToInitialize(imgui_adaptors::ContextHandler pContext,
                                     sdl_adapters::WindowHandler pWindow,
                                     sdl_adapters::RendererHandler pRenderer) {
  std::lock_guard lk{m_FlagLock};
  if (m_isInitialized) {
    throw std::logic_error(
        "ImGui backend can not be initialized more than once");
  }
  m_isInitialized = true;
  m_pWindow = pWindow;     // ownership until destruction
  m_pRenderer = pRenderer; // ownership until destruction
  m_pContext = pContext;   // ownership until destruction
  ImGui_ImplSDL2_InitForSDLRenderer(pWindow.get(), pRenderer.get());
  ImGui_ImplSDLRenderer2_Init(pRenderer.get());
}

BackendManager::~BackendManager() {
  std::lock_guard lk{m_FlagLock};
  if (m_isInitialized) {
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
  }
}

WindowGuard::WindowGuard(const char *name, bool *p_open,
                         ImGuiWindowFlags flags) noexcept {
  ImGui::Begin(name, p_open, flags);
}

WindowGuard::~WindowGuard() { ImGui::End(); }

}; // namespace imgui_adaptors