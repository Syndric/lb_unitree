cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)
add_link_options(-no-pie)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

include_directories(include)
link_directories(lib/cpp/${ARCH})

option(PYTHON_BUILD "build python wrapper" OFF)
if(PYTHON_BUILD)
  add_subdirectory(python_wrapper)
endif()

set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 14) # Requires C++14 for this GUI app

find_package(catkin QUIET)
if(${catkin_FOUND})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  )
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# --- Existing Examples ---
add_executable(example_position example/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity example/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque example/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk example/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_joystick example/example_joystick.cpp)
target_link_libraries(example_joystick ${EXTRA_LIBS})

add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor ${EXTRA_LIBS})


# ===================================================================
# --- NEW SECTION FOR GUI MONITOR ---
# ===================================================================

# --- 1. Find GLFW and OpenGL ---
# These are required by the ImGui backend
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# --- 2. Define ImGui source files ---
# This assumes you have an 'imgui' directory alongside your 'example' dir
# containing the Dear ImGui source files.
set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# --- 3. Add ImGui include directory ---
include_directories(${IMGUI_DIR})
include_directories(${IMGUI_DIR}/backends)

# --- 4. Add the executable ---
add_executable(monitor_gui src/monitor_gui.cpp ${IMGUI_SOURCES})

# --- 5. Link all libraries ---
target_link_libraries(monitor_gui
  ${EXTRA_LIBS}     # unitree_legged_sdk and pthread
  glfw              # The windowing library
  ${OPENGL_LIBRARIES} # OpenGL
)

# ===================================================================

# install
install(TARGETS
  example_position example_velocity example_torque example_walk example_joystick monitor monitor_gui
  DESTINATION bin/unitree)
install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/unitree
  USE_SOURCE_PERMISSIONS)