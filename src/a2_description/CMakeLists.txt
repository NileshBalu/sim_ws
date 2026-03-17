cmake_minimum_required(VERSION 3.16)
project(a2_description)

find_package(ament_cmake REQUIRED)

# Install the a2 directory to the share folder
install(DIRECTORY a2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()