cmake_minimum_required(VERSION 3.16)
project(a2_mujoco)

# --- ROS 2 ADDITION ---
find_package(ament_cmake REQUIRED)
# ----------------------

# Set C++ standard. Build type flags (e.g., -O3, -g) are handled by CMake/ament.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(ament_index_cpp REQUIRED)

file(GLOB SIM_SRC
  src/joystick/joystick.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

add_executable(a2_mujoco ${SIM_SRC} src/main.cc)
set_target_properties(a2_mujoco PROPERTIES
  # This ensures the installed executable can find its shared library dependencies.
  # $ORIGIN refers to the executable's own directory.
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN:/opt/unitree_robotics/lib"
)

# Force the linker to use RPATH instead of RUNPATH. This ensures the Unitree 
# libraries are always loaded *before* the ROS 2 libraries in LD_LIBRARY_PATH.
target_link_options(a2_mujoco PRIVATE "-Wl,--disable-new-dtags")

# Use target_* commands to scope includes and links to specific targets.
target_include_directories(a2_mujoco PRIVATE
  mujoco/include
  mujoco/simulate
  src/lodepng
)

target_link_directories(a2_mujoco PRIVATE mujoco/lib)

target_link_libraries(a2_mujoco PRIVATE
  pthread # Note: Using target_link_libraries is preferred over link_libraries in ROS 2
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
  ament_index_cpp::ament_index_cpp
)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

# --- ROS 2 INSTALLATION RULES ---
# This moves the binaries to the 'lib' folder of your install workspace
install(TARGETS
  a2_mujoco
  jstest
  DESTINATION lib/${PROJECT_NAME}
)

# Install the local MuJoCo libraries alongside the executable
install(DIRECTORY mujoco/lib/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "libmujoco.so*"
)

# Install the config file
install(FILES config.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
# ----------------------