cmake_minimum_required(VERSION 3.0.2)
project(camera_test)

find_package(OpenCV)

add_executable(${PROJECT_NAME} NIRS.cpp)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
add_custom_command(TARGET ${PROJECT_NAME}
                   COMMAND ln -sfv ${CMAKE_SOURCE_DIR}/parameters.yml ${PROJECT_BINARY_DIR}
)
