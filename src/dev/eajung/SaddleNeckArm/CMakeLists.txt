link_directories(${LIB_DIR})

link_libraries(tgcreator core Adapters)

add_executable(AppSaddleNeckArm
AppSaddleNeckArm.cpp
SaddleNeckArmModel.cpp
)
