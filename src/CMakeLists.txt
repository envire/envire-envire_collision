set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

rock_library(envire_collision
    SOURCES ODECollisionBase.cpp
    HEADERS Exceptions.hpp
            ODECollisionBase.hpp
            ODECollisionHeader.hpp
            ODECollision.hpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG ode envire_core
    LIBS Threads::Threads)
