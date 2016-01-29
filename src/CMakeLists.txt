rock_library(envire_collision
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(envire_collision_bin Main.cpp
    DEPS envire_collision)

