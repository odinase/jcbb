cmake_minimum_required(VERSION 3.10)
project(jcbb)

set (CMAKE_CXX_STANDARD 17)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(Boost REQUIRED)

find_package (glog 0.6.0 REQUIRED)

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

include_directories(
    include
    eigen3
)

add_library(utils
    src/utils
)

add_library(hypothesis
src/Hypothesis
)

target_link_libraries(hypothesis
gtsam
)

add_library(marginals
    src/MarginalMocks.cpp
)

target_link_libraries(marginals
    gtsam
    Eigen3::Eigen
)

add_library(jcbb
    src/jcbb
)

target_link_libraries(jcbb
    utils
    ${Boost_LIBRARIES}
    gtsam
    hypothesis
    marginals
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


add_executable(testing
    testing.cpp
    src/file_parser.cpp
)

target_link_libraries(testing
    marginals
    gtsam
    Eigen3::Eigen
    glog::glog
)