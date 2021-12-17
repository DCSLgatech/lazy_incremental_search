cmake_minimum_required(VERSION 2.8.3)
project(lgls)
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

#================================================================================
# Compiler settings
#================================================================================
# Enforce the use of C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wuninitialized -Wpedantic")

add_compile_options(-Wall -Wextra -Wpedantic)
if(GENERALIZED_LAZY_SEARCH_TREAT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

#================================================================================
# Link Libraries
#================================================================================

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/datastructures/Graph.cpp
  src/datastructures/Queue.cpp
  src/datastructures/State.cpp
  # src/GLS.cpp
  src/LGLS.cpp
  src/BLGLS.cpp
  src/LPAstar.cpp
  src/sampler/HaltonSequence.cpp
  src/TLPAstar.cpp
  # src/GTLPAstar.cpp
  src/event/Event.cpp
  src/event/ShortestPathEvent.cpp
  src/event/ConstantDepthEvent.cpp
  src/selector/Selector.cpp
  src/selector/AlternateSelector.cpp
  src/selector/BackwardSelector.cpp
  src/selector/FailFastSelector.cpp
  src/selector/ForwardSelector.cpp
)

#================================================================================
# Example Scripts
#================================================================================

# Correctness Test Script

add_executable(test_LPAstar examples/test_LPAstar.cpp)
target_link_libraries(test_LPAstar ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_LPAstar RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_TLPAstar examples/test_TLPAstar.cpp)
target_link_libraries(test_TLPAstar ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_TLPAstar RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(test_GTLPAstar examples/test_GTLPAstar.cpp)
#target_link_libraries(test_GTLPAstar ${PROJECT_NAME} ${OpenCV_LIBS})
#install(TARGETS test_GTLPAstar RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_LGLS examples/test_LGLS.cpp)
target_link_libraries(test_LGLS ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_LGLS RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_LGLS_inc examples/test_LGLS_inc.cpp)
target_link_libraries(test_LGLS_inc ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_LGLS_inc RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_BLGLS examples/test_BLGLS.cpp)
target_link_libraries(test_BLGLS ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_BLGLS RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_BLGLS_inc examples/test_BLGLS_inc.cpp)
target_link_libraries(test_BLGLS_inc ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_BLGLS_inc RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#================================================================================
# Installation
#================================================================================

# install(TARGETS lgls EXPORT lglsConfig
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#  )

#install(DIRECTORY include/ DESTINATION include)

## This makes the project importable from the install directory
# install(EXPORT lglsConfig DESTINATION share/lgls/cmake)

## This makes the project importable from the build directory
# export(TARGETS lgls FILE lglsConfig.cmake)

install(TARGETS lgls EXPORT
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/ DESTINATION include)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/lgls")

#================================================================================
# Formatting
#================================================================================
include(ClangFormat)

clang_format_setup(VERSION 3.8)

if (CLANG_FORMAT_EXECUTABLE)
  FILE(GLOB_RECURSE lgls_headers "include/*.hpp")
  FILE(GLOB_RECURSE lgls_srcs "include/*.cpp")
  clang_format_add_sources(${lgls_headers})
  clang_format_add_sources(${lgls_srcs})

  clang_format_add_targets()
endif()

#================================================================================
# IDE Comfort
#================================================================================

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})
