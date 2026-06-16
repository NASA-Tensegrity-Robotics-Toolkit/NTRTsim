link_directories(${LIB_DIR})

add_library(TimestepTest SHARED
			tsTestRig.cpp)

target_link_libraries(TimestepTest
    controllers
    tgcreator
    util
    sensors
    core
    terrain
    tgOpenGLSupport)

add_executable(AppTSTest
    tsTestRig.cpp
    AppTimestepTest.cpp
)

target_link_libraries(AppTSTest
    controllers
    tgcreator
    util
    sensors
    core
    terrain
    tgOpenGLSupport)

set_target_properties(AppTSTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

ntrt_add_runme(AppTSTest DESCRIPTION "Run the motor timestep test demo")
