link_directories(${LIB_DIR})

link_libraries(tgcreator core Adapters)

add_executable(AppSaddleArm
AppSaddleArm.cpp
SaddleArmModel.cpp
)
