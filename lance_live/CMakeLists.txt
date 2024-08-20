cmake_minimum_required(VERSION 3.5)
project(lance_live)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
