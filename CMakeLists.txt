cmake_minimum_required( VERSION 2.8 )
project( multiple_sensor_fusion)

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set( Sophus_LIBRARIES libSophus.so)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen
include_directories( "/usr/include/eigen3" )

# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# g2o 
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

# boost
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#gtsam
find_package(GTSAM REQUIRED)
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})

#GeographicLib
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIR})

#target_link_libraries(sensor_fusion_gtsam ${boost_LIBRARIES})

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
target_link_libraries(multiple_sensor_fusion ${Boost_LIBRARIES} -lgtsam -ltbb)
target_link_libraries(multiple_sensor_fusion ${GeographicLib_LIBRARIES})


