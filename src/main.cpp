#ifndef linux
static_assert(false, "This code is valid for Ubuntu x64 linux");
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image.h>
#include <stb_image_write.h>
#include <SDL_keycode.h>
#include <SDL.h>
#include <LiteMath/LiteMath.h>
#include <LiteMath/Image2d.h>

#include <cstdint>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unistd.h>

#include "mesh.h"

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

struct SdfGrid
{
  uint3 size;
  std::vector<float> data; // size.x*size.y*size.z values
};

void save_sdf_grid(const SdfGrid &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, 3 * sizeof(unsigned));
  fs.write((const char *)scene.data.data(), scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.flush();
  fs.close();
}

void load_sdf_grid(SdfGrid &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  fs.read((char *)&scene.size, 3 * sizeof(unsigned));
  scene.data.resize(scene.size.x * scene.size.y * scene.size.z);
  fs.read((char *)scene.data.data(), scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.close();
}

struct SdfOctreeNode
{
  float values[8];
  unsigned offset; // offset for children (they are stored together). 0 offset means it's a leaf
};

struct SdfOctree
{
  std::vector<SdfOctreeNode> nodes;
};

void save_sdf_octree(const SdfOctree &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  size_t size = scene.nodes.size();
  fs.write((const char *)&size, sizeof(unsigned));
  fs.write((const char *)scene.nodes.data(), size * sizeof(SdfOctreeNode));
  fs.flush();
  fs.close();
}

void load_sdf_octree(SdfOctree &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.nodes.resize(sz);
  fs.read((char *)scene.nodes.data(), scene.nodes.size() * sizeof(SdfOctreeNode));
  fs.close();
}

struct AppData
{
  int width;
  int height;
  SdfGrid loaded_grid;
  int z_level = 32;
};

void draw_sdf_grid_slice(const SdfGrid &grid, int z_level, int voxel_size,
                         int width, int height, std::vector<uint32_t> &pixels)
{
  constexpr uint32_t COLOR_EMPTY = 0xFF333333;  // dark gray
  constexpr uint32_t COLOR_FULL = 0xFFFFA500;   // orange
  constexpr uint32_t COLOR_BORDER = 0xFF000000; // black

  for (int y = 0; y < grid.size.y; y++)
  {
    for (int x = 0; x < grid.size.x; x++)
    {
      int index = x + y * grid.size.x + z_level * grid.size.x * grid.size.y;
      uint32_t color = grid.data[index] < 0 ? COLOR_FULL : COLOR_EMPTY;
      for (int i = 0; i <= voxel_size; i++)
      {
        for (int j = 0; j <= voxel_size; j++)
        {
          // flip the y axis
          int pixel_idx = (x * voxel_size + i) + ((height - 1) - (y * voxel_size + j)) * width;
          if (i == 0 || i == voxel_size || j == 0 || j == voxel_size)
            pixels[pixel_idx] = COLOR_BORDER;
          else
            pixels[pixel_idx] = color;
        }
      }
    }
  }
}

void draw_frame(const AppData &app_data, std::vector<uint32_t> &pixels)
{
  //your rendering code goes here
  std::fill_n(pixels.begin(), app_data.width * app_data.height, 0xFFFFFFFF);
  int voxel_size = std::min((app_data.width - 1) / app_data.loaded_grid.size.x,
                            (app_data.height - 1) / app_data.loaded_grid.size.y);

  draw_sdf_grid_slice(app_data.loaded_grid, app_data.z_level, voxel_size, app_data.width, app_data.height, pixels);
}

void save_frame(const char* filename, const std::vector<uint32_t>& frame, uint32_t width, uint32_t height)
{
  LiteImage::Image2D<uint32_t> image(width, height, frame.data());

  // Convert from ARGB to ABGR
  for (uint32_t i = 0; i < width * height; i++) {
    uint32_t& pixel = image.data()[i];
    auto a = (pixel & 0xFF000000);
    auto r = (pixel & 0x00FF0000) >> 16;
    auto g = (pixel & 0x0000FF00);
    auto b = (pixel & 0x000000FF) << 16;
    pixel = a | b | g | r;
  }

  if (LiteImage::SaveImage(filename, image))
    std::cout << "Image saved to " << filename << std::endl;
  else
    std::cout << "Image could not be saved to " << filename << std::endl;
  // If you want a slightly more low-level API, You can manually do:
  //    stbi_write_png(filename, width, height, 4, (unsigned char*)frame.data(), width * 4)
}

// You must include the command line parameters for your main function to be recognized by SDL
int main(int argc, char **args)
{
  std::filesystem::path exec_path;
  {
    char path_cstr[PATH_MAX+1] = {};
    std::ignore = readlink("/proc/self/exe", path_cstr, PATH_MAX);
    exec_path = path_cstr;
  }
  auto project_path = exec_path.parent_path().parent_path();
  std::filesystem::current_path(project_path);
  auto resources_path = project_path / "resources";

  const int SCREEN_WIDTH = 1280;
  const int SCREEN_HEIGHT = 720;

  // Pixel buffer (RGBA format)
  std::vector<uint32_t> pixels(SCREEN_WIDTH * SCREEN_HEIGHT, 0xFFFFFFFF); // Initialize with white pixels
  AppData app_data;
  app_data.width = SCREEN_WIDTH;
  app_data.height = SCREEN_HEIGHT;
  

  load_sdf_grid(app_data.loaded_grid, resources_path / "example_grid.grid");
  // Initialize SDL. SDL_Init will return -1 if it fails.
  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
  {
    std::cerr << "Error initializing SDL: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create our window
  SDL_Window *window = SDL_CreateWindow("Triangle/SDF Viewer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

  // Make sure creating the window succeeded
  if (!window)
  {
    std::cerr << "Error creating window: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create a renderer
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
  if (!renderer)
  {
    std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  // Create a texture
  SDL_Texture *texture = SDL_CreateTexture(
      renderer,
      SDL_PIXELFORMAT_ARGB8888,    // 32-bit RGBA format
      SDL_TEXTUREACCESS_STREAMING, // Allows us to update the texture
      SCREEN_WIDTH,
      SCREEN_HEIGHT);

  if (!texture)
  {
    std::cerr << "Texture could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  SDL_Event ev;
  bool running = true;

  // Main loop
  while (running)
  {
    // Event loop
    while (SDL_PollEvent(&ev) != 0)
    {
      // check event type
      switch (ev.type)
      {
      case SDL_QUIT:
        // shut down
        running = false;
        break;
      case SDL_KEYDOWN:
        // test keycode
        switch (ev.key.keysym.sym)
        {
        //W and S keys to change the slice of the grid currently rendered
        case SDLK_w:
          app_data.z_level = std::min<int>(app_data.z_level + 1, app_data.loaded_grid.size.z - 1);
          std::cout << "z_level=" << app_data.z_level << std::endl;
          break;
        case SDLK_s:
          app_data.z_level = std::max<int>(app_data.z_level - 1, 0);
          std::cout << "z_level=" << app_data.z_level << std::endl;
          break;
        //ESC to exit 
        case SDLK_ESCAPE:
          running = false;
          break;
          // etc
        }
        break;
      }
    }

    // Update pixel buffer
    draw_frame(app_data, pixels);

    // Update the texture with the pixel buffer
    SDL_UpdateTexture(texture, nullptr, pixels.data(), SCREEN_WIDTH * sizeof(uint32_t));

    // Clear the renderer
    SDL_RenderClear(renderer);

    // Copy the texture to the renderer
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);

    // Update the screen
    SDL_RenderPresent(renderer);
  }

  // Destroy the window. This will also destroy the surface
  SDL_DestroyWindow(window);

  // Quit SDL
  SDL_Quit();

  // End the program
  return 0;
}
