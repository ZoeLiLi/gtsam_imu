cmake_minimum_required( VERSION 2.8 )
project( sensor_fusion_gtsam )

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
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#gtsam
find_package(GTSAM REQUIRED)
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})

#target_link_libraries(sensor_fusion_gtsam ${boost_LIBRARIES})

#set (SOURCE_FOLDER ${PROJECT_SOURCE_DIR}/src)
#set (SOURCE_FILE ${SOURCE_FOLDER}/data_load.cpp
#${SOURCE_FOLDER}/sensor_fusion_gtsam.cpp)
							  
add_executable( sensor_fusion_gtsam sensor_fusion_gtsam.cpp )
target_link_libraries(sensor_fusion_gtsam ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS sensor_fusion_gtsam RUNTIME DESTINATION bin)


