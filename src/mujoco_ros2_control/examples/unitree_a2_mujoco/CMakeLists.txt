cmake_minimum_required(VERSION 3.8)
project(unitree_a2_mujoco)

# find dependencies
find_package(ament_cmake REQUIRED)

###########
# INSTALL #
###########

# Install the directories needed for the ROS 2 stack
install(DIRECTORY launch config urdf meshes
        DESTINATION share/${PROJECT_NAME}
)

# Install the MuJoCo XML file specifically (located in package root)
install(FILES 
        a2.xml
        scene_terrain.xml 
        DESTINATION share/${PROJECT_NAME}
)

ament_package()