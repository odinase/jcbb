cmake_minimum_required(VERSION 3.10)
project(jcbb)

set (CMAKE_CXX_STANDARD 17)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.50 REQUIRED)
find_package (glog 0.6.0 REQUIRED)

include_directories(
    include
    eigen3
)

add_library(utils
    src/utils
)

add_library(jcbb
    src/jcbb
)

target_link_libraries(jcbb
    utils
)

add_executable(main
    main.cpp
    src/file_parser.cpp
)

target_link_libraries(main
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    glog::glog
    jcbb
    utils
)