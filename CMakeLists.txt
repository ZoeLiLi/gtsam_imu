cmake_minimum_required( VERSION 3.0 )
project( multiple_sensor_fusion)

set( CMAKE_BUILD_TYPE "debug" )
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

add_executable( multiple_sensor_fusion ${SOURCE_FILES} ${HEAD_FILES})

SET(LINK_MODULE1_LIBS boost_serialization boost_system boost_filesystem boost_thread boost_program_options boost_date_time boost_timer boost_chrono boost_regex)
FOREACH(module1 ${LINK_MODULE1_LIBS})
	add_library(${module1} STATIC IMPORTED)
	target_link_libraries(multiple_sensor_fusion ${PROJECT_SOURCE_DIR}/third_party/lib/boost/${CMAKE_BUILD_TYPE}/lib${module1}.a)
ENDFOREACH(module1)
target_link_libraries(multiple_sensor_fusion pthread)


SET(LINK_module2_LIBS gtsam Geographic Sophus)
FOREACH(module2 ${LINK_module2_LIBS})
	add_library(${module2} STATIC IMPORTED)
	target_link_libraries(multiple_sensor_fusion ${PROJECT_SOURCE_DIR}/third_party/lib/lib${module2}.a)
ENDFOREACH(module2)



