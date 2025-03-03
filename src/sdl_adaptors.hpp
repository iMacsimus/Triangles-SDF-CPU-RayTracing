#pragma once

#include <cinttypes>
#include <exception>
#include <memory>
#include <mutex>
#include <string_view>

#include <SDL2/SDL.h>

namespace sdl_adapters {

class SDLManager {
public:
  SDLManager(const SDLManager &) = delete;
  SDLManager(SDLManager &&) = delete;
  SDLManager &operator=(const SDLManager &) = delete;
  SDLManager &operator=(SDLManager &&) = delete;

public:
  static SDLManager &getInstance() noexcept;
  void tryToInitialize(uint32_t flags);

private:
  SDLManager() noexcept = default;
  bool m_isInitialized = false;
  std::mutex m_FlagLock;

public:
  ~SDLManager();
};

using WindowHandler = std::shared_ptr<SDL_Window>;
using RendererHandler = std::shared_ptr<SDL_Renderer>;
using TextureHandler = std::shared_ptr<SDL_Texture>;

WindowHandler createWindow(std::string_view title, int x, int y, int w, int h,
                           uint32_t flags);

RendererHandler createRenderer(WindowHandler pWindow, int index,
                               uint32_t flags);

TextureHandler createTexture(RendererHandler pRenderer, uint32_t format,
                             int access, int w, int h);

} // namespace sdl_adapters
