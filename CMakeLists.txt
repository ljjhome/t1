cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(apollo)

set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
    roscpp
)
catkin_package(
    CATKIN_DEPENDS roscpp
)

#-------include -------------
include_directories(${PROJECT_SOURCE_DIR})

#------ protobuf-------
set(
    protobuf_lib
        /home/ljj/program/protobuf_install/lib/libprotobuf.so 
)
set(
    protobuf_include_dir
        /home/ljj/program/protobuf_install/include
)

#-------- eigen3 -----------
# include_directories(/usr/include/eigen3)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message("!!!!!!!!!!!!!!!!")
message("${Eigen_INCLUDE_DIR}")

#---------- glog ----------------
set(glog_lib /usr/lib/x86_64-linux-gnu/libglog.so)

#---------- gtest----------------
set(gtest_include_dir /home/ljj/program/gtest_install/include )
set(gtest_lib /home/ljj/program/gtest_install/lib/libgtest.a)
set(gtest_main_lib /home/ljj/program/gtest_install/lib/libgtest_main.a)

#-----------gmock -------------------
set(gmock_include_dir /home/ljj/program/gtest_install/include )
set(gmock_lib /home/ljj/program/gtest_install/lib/libgmock.a)
set(gmock_main_lib /home/ljj/program/gtest_install/lib/libgmock_main.a)

#---------- gflags -----------------
find_package(gflags REQUIRED)

#----------- absl -------------
set(absl_cmake_dir /home/ljj/program/abseil_install_shared/lib/cmake/absl)
include_directories(/home/ljj/program/abseil_install_shared/include)
find_package(absl REQUIRED PATHS ${absl_cmake_dir})


#-----------json --------------
set(nlohmann_json_include_dir /home/ljj/program/nlohmann_json/include)
find_package(nlohmann_json REQUIRED PATHS /home/ljj/program/nlohmann_json/lib/cmake/nlohmann_json)


#--------- osqp -------------
find_package(osqp REQUIRED)



#_---------- tinyxml2 ----------------
# # find_package(tinyxml2 REQUIRED PATHS /usr/local/lib/cmake/tinyxml2)
# set(tinyxml2_lib /usr/local/lib/libtinyxml2.so)
#-------------libproj----------------
set(proj_lib /usr/lib/x86_64-linux-gnu/libproj.so)

add_subdirectory(modules)
add_subdirectory(cyber)