cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(SdfTask CXX)

find_package(SDL2 REQUIRED)
find_package(OpenMP REQUIRED)

add_library(
  LiteMath
    ${CMAKE_SOURCE_DIR}/external/LiteMath/Image2d.cpp)
target_include_directories(LiteMath PUBLIC ${CMAKE_SOURCE_DIR}/external/)

add_library(
  ImGui_SDL
    ${CMAKE_SOURCE_DIR}/external/ImGui/imgui_demo.cpp
    ${CMAKE_SOURCE_DIR}/external/ImGui/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/external/ImGui/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/external/ImGui/imgui.cpp
    ${CMAKE_SOURCE_DIR}/external/ImGui/imgui_widgets.cpp
    ${CMAKE_SOURCE_DIR}/external/ImGui/backends/imgui_impl_sdlrenderer2.cpp 
    ${CMAKE_SOURCE_DIR}/external/ImGui/backends/imgui_impl_sdl2.cpp)
target_include_directories(
  ImGui_SDL PUBLIC 
    ${CMAKE_SOURCE_DIR}/external/ImGui 
    ${CMAKE_SOURCE_DIR}/external/ImGui/backends 
    ${SDL2_INCLUDE_DIRS})

set(
  SRC_CORE 
    ${CMAKE_SOURCE_DIR}/src/core/mesh.cpp
    ${CMAKE_SOURCE_DIR}/src/core/mesh.h 
    ${CMAKE_SOURCE_DIR}/src/core/tiny_obj_loader.h)
set( 
  SRC_VIEWER
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/sdl_adaptors.cpp
    ${CMAKE_SOURCE_DIR}/src/imgui_adaptors.cpp
    ${CMAKE_SOURCE_DIR}/src/quaternion.cpp
    ${CMAKE_SOURCE_DIR}/src/camera.cpp
    ${CMAKE_SOURCE_DIR}/src/triangles_raytracing.cpp)

enable_language(ISPC)
set(CMAKE_ISPC_FLAGS "${CMAKE_ISPC_FLAGS} --pic --werror")
add_library(ispc_ray_pack src/ray_pack.ispc)   

set(APP_NAME GUIApplication)
add_executable(
  ${APP_NAME}
    ${SRC_CORE}
    ${SRC_VIEWER})
target_link_libraries(
  ${APP_NAME} 
    ${SDL2_LIBRARIES} 
    ispc_ray_pack
    LiteMath
    ImGui_SDL
    OpenMP::OpenMP_CXX)
target_include_directories(
    ${APP_NAME} PUBLIC 
      ${SDL2_INCLUDE_DIRS} 
      ${CMAKE_SOURCE_DIR}/src/core
      ${CMAKE_SOURCE_DIR}/src/
      ${CMAKE_SOURCE_DIR}/external/stb/)
target_compile_options(${APP_NAME} PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)