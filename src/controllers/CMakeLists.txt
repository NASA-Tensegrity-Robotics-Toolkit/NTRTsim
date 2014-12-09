project(controllers)

add_library( ${PROJECT_NAME} SHARED
#PretensionController.cpp - depricated?
tgBasicController.cpp
tgImpedanceController.cpp
tgPIDController.cpp
tgTensionController.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} core)

