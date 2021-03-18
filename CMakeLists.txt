cmake_minimum_required(VERSION 3.5)
project(bio_ik)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time chrono)

find_package(OpenMP)
# the specific flag is not yet present in cmake 2.8.12
if (OpenMP_CXX_FOUND OR OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    add_compile_options(${OpenMP_CXX_FLAGS})
    if (NOT OpenMP_CXX_LIBRARIES)
        # cmake 2.8.12 does not yet specify the library - assume we might need libgomp
        set(OpenMP_LIBS gomp)
    else ()
        set(OpenMP_LIBS ${OpenMP_CXX_LIBRARIES})
    endif ()
    if (MSVC)
        add_compile_options(-openmp:experimental)
    endif()
else ()
    message(WARNING "OPENMP NOT FOUND. You will suffer performance loss.")
    set(OpenMP_LIBS)
endif ()

option(USE_FANN "build the neural-network-based IK solver (experimental)" OFF)
if (USE_FANN)
    find_library(FANN_LIBRARIES NAMES fann)
    find_path(FANN_INCLUDE_DIRS NAMES fann.h)
    if (NOT FANN_INCLUDE_DIRS OR NOT FANN_LIBRARIES)
        message(FATAL_ERROR "Neural network solver requested, but libfann was not found.")
    else ()
        message("Found libfann: ${FANN_LIBRARIES} / ${FANN_INCLUDE_DIRS}")
    endif ()
else ()
    set(FANN_LIBRARIES)
    set(FANN_INCLUDE_DIRS)
endif ()

option(USE_CPPOPTLIB "Include gradient-based solvers from CppNumericalSolvers (bio_ik also provides its own solver)" OFF)
if (USE_CPPOPTLIB)
    find_path(CPPOPTLIB_INCLUDE_DIRS
            NAMES cppoptlib/solver/bfgssolver.h
            HINTS ../../CppNumericalSolvers/include)
    if (NOT CPPOPTLIB_INCLUDE_DIRS)
        message(FATAL_ERROR "cppoptlib support requested, but the headers could not be found.")
    else ()
        message("Found cppoptlib: ${CPPOPTLIB_INCLUDE_DIRS}")
    endif ()
    add_definitions(-DENABLE_CPP_OPTLIB)
else ()
    set(CPPOPTLIB_INCLUDE_DIRS)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-frecord-gcc-switches)
endif ()

add_compile_options($<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU>>:-O3>)
add_compile_options($<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU>>:-ftree-vectorize>)
add_compile_options($<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU>>:-ffast-math>)

add_compile_options($<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2>)

include_directories(
        include
        ${FANN_INCLUDE_DIRS}
        ${CPPOPTLIB_INCLUDE_DIRS}
)

set(SOURCES
        src/goal_types.cpp
        include/bio_ik/bio_ik_plugin.hpp
        src/problem.cpp

        src/ik_test.cpp
        src/ik_gradient.cpp
        src/ik_evolution_1.cpp
        src/ik_evolution_2.cpp
        src/bio_ik_plugin.cpp)

if (USE_FANN)
    list(APPEND SOURCES src/ik_neural.cpp)
endif ()

if (USE_CPPOPTLIB)
    list(APPEND SOURCES src/ik_cppoptlib.cpp)
endif ()

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_compile_definitions(${PROJECT_NAME} PRIVATE BIO_IK_BUILDING)

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        moveit_core
        moveit_ros_planning
        pluginlib
        rclcpp
        tf2
        tf2_kdl
        tf2_geometry_msgs)

target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_THREAD_LIBRARIES} ${Boost_DATE_TIME_LIBRARIES} ${Boost_CHRONO_LIBRARIES})

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MOVEIT_KDL_KINEMATICS_PLUGIN_BUILDING_DLL")

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${FANN_LIBRARIES}
)

if("${CMAKE_SYSTEM}" MATCHES "Linux")
    target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenMP_LIBS}  
        -static-libgcc
        -static-libstdc++
        )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(pluginlib)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(Boost)
ament_package()