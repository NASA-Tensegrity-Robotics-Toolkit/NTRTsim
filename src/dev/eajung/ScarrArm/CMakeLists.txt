link_directories(${LIB_DIR})

link_libraries(tgcreator core Adapters)

add_executable(AppScarrArm
ScarrArmModel.cpp
ScarrArmController.cpp
AppScarrArm.cpp
)
