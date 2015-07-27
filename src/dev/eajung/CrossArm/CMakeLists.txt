link_directories(${LIB_DIR})

link_libraries(tgcreator core Adapters)

add_executable(AppCrossArm
CrossArmModel.cpp
CrossArmController.cpp
AppCrossArm.cpp
)
