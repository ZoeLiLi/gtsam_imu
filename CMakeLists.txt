cmake_minimum_required( VERSION 3.0 )
project( multiple_sensor_fusion)

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include_directories(${PROJECT_SOURCE_DIR}/third_party/include)

set (SOURCE_FOLDER ${PROJECT_SOURCE_DIR}/src)
set (HEADER_FILES ${SOURCE_FOLDER}/alignment.h
        ${SOURCE_FOLDER}/constant.h
        ${SOURCE_FOLDER}/gnss_class.h
        ${SOURCE_FOLDER}/imu_class.h
        ${SOURCE_FOLDER}/mm_feedback_class.h
        ${SOURCE_FOLDER}/odometry_class.h
        ${SOURCE_FOLDER}/sensor_factors.h
        ${SOURCE_FOLDER}/sensor_fusion.h
        ${SOURCE_FOLDER}/factors/velocity_factor.h
        ${SOURCE_FOLDER}/factors/gnss_factor.h)

set (SOURCE_FILES ${SOURCE_FOLDER}/alignment.cpp
        ${SOURCE_FOLDER}/gnss_class.cpp
        ${SOURCE_FOLDER}/imu_class.cpp
        ${SOURCE_FOLDER}/main.cpp
        ${SOURCE_FOLDER}/mm_feedback_class.cpp
        ${SOURCE_FOLDER}/odometry_class.cpp
        ${SOURCE_FOLDER}/sensor_factors.cpp
        ${SOURCE_FOLDER}/sensor_fusion.cpp
        ${SOURCE_FOLDER}/factors/velocity_factor.cpp
        ${SOURCE_FOLDER}/factors/gnss_factor.cpp)

include_directories(${SOURCE_FOLDER})
link_directories(${PROJECT_SOURCE_DIR}/third_party/lib)
link_directories(${PROJECT_SOURCE_DIR}/third_party/lib/boost/debug)
link_directories(${PROJECT_SOURCE_DIR}/third_party/lib/boost/release)

add_executable( multiple_sensor_fusion ${SOURCE_FILES} ${HEAD_FILES})

target_link_libraries(multiple_sensor_fusion -lgtsam -lGeographic -lSophus -ltbb)
target_link_libraries(multiple_sensor_fusion -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -lboost_program_options -lboost_date_time -lboost_timer -lboost_chrono -lboost_regex)

#target_link_libraries(multiple_sensor_fusion ${Boost_LIBRARIES} -lgtsam -ltbb)
#target_link_libraries(multiple_sensor_fusion ${GeographicLib_LIBRARIES})


