set(TARGET_NAME camera_calibration)

find_package(Eigen3 REQUIRED HINTS "D:/ThirdParty/Eigen3/share/eigen3/cmake")
find_package(Ceres REQUIRED HINTS "D:/ThirdParty/Ceres/lib/cmake/Ceres")
find_package(OpenCV REQUIRED HINTS "D:/ThirdParty/opencv480/opencv/build/x64/vc16/lib")
find_package(Boost COMPONENTS filesystem system thread program_options REQUIRED)
# for qt
# set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE CPP_HEAD_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE CPP_SOUCE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${CPP_HEAD_FILES} ${CPP_SOUCE_FILES})

add_executable(${TARGET_NAME} ${CPP_HEAD_FILES} ${CPP_SOUCE_FILES})

set_target_properties(${TARGET_NAME} PROPERTIES CXX_STANDARD 17 OUTPUT_NAME ${PROJECT_NAME})
set_target_properties(${TARGET_NAME} PROPERTIES FOLDER Apps)

target_compile_options(${TARGET_NAME} PUBLIC "$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/WX->")

target_link_libraries(${TARGET_NAME} PRIVATE Eigen3::Eigen opencv_world 
Boost::system
Boost::filesystem
Boost::dynamic_linking
Boost::thread
Boost::program_options
ceres
)

target_include_directories(
  ${TARGET_NAME} 
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
)