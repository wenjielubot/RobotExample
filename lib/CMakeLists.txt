set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")


find_package(Qt5Widgets REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)


include_directories(
#${Qt5Core_INCLUDE_DIRS}
${Qt5Widgets_INCLUDE_DIRS}
)

#add_library(SimulatorApp SimulatorApp.cpp)

add_library (Simulator utl.cpp Simulator.cpp SimulatorApp.cpp)


