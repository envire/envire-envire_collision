# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.0)
project(envire_collision VERSION 0.1)
find_package(Rock)
set(ROCK_TEST_ENABLED ON)
rock_init()

find_package(Boost COMPONENTS system)
rock_standard_layout()
