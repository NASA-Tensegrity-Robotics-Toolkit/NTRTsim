project(sensors)

link_directories(${LIB_DIR})

link_libraries(${PROJECT_NAME} util core tgOpenGLSupport)

add_library( ${PROJECT_NAME} SHARED
    tgDataLogger.cpp
    tgDataObserver.cpp
)
