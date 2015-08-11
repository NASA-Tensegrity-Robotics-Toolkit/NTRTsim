link_directories(${LIB_DIR})

link_libraries(tgcreator core Adapters)

add_executable(AppSaddleArmNew
AppSaddleArm.cpp
SaddleArmModel.cpp
SaddleArmController.cpp
)
