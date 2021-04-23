cmake_minimum_required (VERSION 3.10)
project (SPI_1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(main2 main2.cpp)
target_link_libraries(main2 Threads::Threads)


