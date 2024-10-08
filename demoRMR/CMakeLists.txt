cmake_minimum_required(VERSION 3.22)
project(DemoRMR LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(OpenCV REQUIRED)
find_package(SDL REQUIRED)

add_subdirectory(robot)
add_subdirectory(QJoysticks-master)
add_subdirectory(kd-tree)

add_definitions(-DMAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/map.txt")

set(HEADERS
	mainwindow.h
	pidcontroller.h
	robotTrajectoryController.h
	lidarMapper.h
	floodPlanner.h
	ParticleFilter.h
)

set(SOURCES
	main.cpp
	mainwindow.cpp
	pidcontroller.cpp
	robotTrajectoryController.cpp
	lidarMapper.cpp
	floodPlanner.cpp
	ParticleFilter.cpp
)

set(UI
	mainwindow.ui
	lidarMapper.ui
)

add_executable(rmr
	${HEADERS}
	${SOURCES}
	${UI}
)

target_link_libraries(
	rmr
PUBLIC
	Qt::Core
	Qt::Widgets
	${OpenCV_LIBS}
	robot
	qjoysticks
	kdtree
)

