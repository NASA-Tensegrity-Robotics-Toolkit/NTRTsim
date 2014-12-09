project(controllers)

add_library( ${PROJECT_NAME} SHARED
PretensionController.cpp
tgBasicController.cpp
tgImpedanceController.cpp
tgPIDController.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} core)

