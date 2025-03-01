#include "sdl_adaptor.hpp"

namespace sdl_adapters {

SDLManager &SDLManager::getInstance() {
  static SDLManager instance;
  return instance;
}

void SDLManager::tryToInitialize(uint32_t flags) {
  std::lock_guard lk{m_FlagLock};
  if (m_isInitialized) {
    throw std::logic_error("SDL can not be initialized more than once");
  } else {
    if (SDL_Init(flags) != 0) {
      throw std::runtime_error(SDL_GetError());
    }
    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif
    m_isInitialized = true;
  }
}

SDLManager::~SDLManager() {
  std::lock_guard lk{m_FlagLock};
  if (m_isInitialized) {
    SDL_Quit();
  }
}

template <typename F, typename... Ts>
static auto callAndCheck(F func, Ts... args) {
  auto res = func(args...);
  if (res == nullptr) {
    throw std::runtime_error(SDL_GetError());
  }
  return res;
}

WindowHandler createWindow(std::string_view title, int x, int y, int w, int h,
                           uint32_t flags) {
  return {callAndCheck(SDL_CreateWindow, title.data(), x, y, w, h, flags),
          SDL_DestroyWindow};
}

RendererHandler createRenderer(WindowHandler pWindow, int index,
                               uint32_t flags) {
  return {callAndCheck(SDL_CreateRenderer, pWindow.get(), index, flags),
          SDL_DestroyRenderer};
}

} // namespace sdl_adapters
