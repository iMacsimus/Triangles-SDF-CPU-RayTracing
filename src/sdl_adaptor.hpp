#pragma once

#include <SDL2/SDL.h>
#include <cinttypes>
#include <exception>
#include <memory>
#include <mutex>
#include <string_view>

namespace sdl_adapters {

class SDLManager {
public:
  SDLManager(const SDLManager &) = delete;
  SDLManager(SDLManager &&) = delete;
  SDLManager &operator=(const SDLManager &) = delete;
  SDLManager &operator=(SDLManager &&) = delete;

public:
  static SDLManager &getInstance();
  void tryToInitialize(uint32_t flags);

private:
  SDLManager() = default;
  bool m_isInitialized = false;
  std::mutex m_FlagLock;

public:
  ~SDLManager();
};

using WindowHandler = std::shared_ptr<SDL_Window>;
using RendererHandler = std::shared_ptr<SDL_Renderer>;

WindowHandler createWindow(std::string_view title, int x, int y, int w, int h,
                           uint32_t flags);

RendererHandler createRenderer(WindowHandler pWindow, int index,
                               uint32_t flags);

} // namespace sdl_adapters
