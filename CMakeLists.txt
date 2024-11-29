cmake_minimum_required(VERSION 3.10)
project(SDLEngine)

set(CMAKE_CXX_STANDARD 14)

# Include SDL2 headers and libraries
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

# Manually include SDL2_image paths
include_directories(/usr/include/SDL2)
set(SDL2_IMAGE_LIB /usr/lib/x86_64-linux-gnu/libSDL2_image.so)

# Manually include SDL2_ttf paths
include_directories(/usr/include/SDL2)
set(SDL2_TTF_LIB /usr/lib/x86_64-linux-gnu/libSDL2_ttf.so)

# Set Box2D paths manually
set(BOX2D_INCLUDE_DIR "/usr/local/include/box2d")
set(BOX2D_LIB "/usr/local/lib/libbox2d.a")  

# Add the executable and link libraries
add_executable(climber main.cpp)
target_link_libraries(climber ${SDL2_LIBRARIES} ${BOX2D_LIB} ${SDL2_IMAGE_LIB} ${SDL2_TTF_LIB})