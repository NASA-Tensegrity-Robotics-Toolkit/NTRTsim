link_libraries(tgcreator core Adapters)

add_executable(AppFlemonsArm
    FlemonsArm/FlemonsArm.cpp
    FlemonsArm/AppFlemonsArm.cpp
)
 
add_executable(AppScarrArmModel
    ScarrArm/ScarrArmModel.cpp
    ScarrArm/AppScarrArm.cpp
    ScarrArm/ScarrArmController.cpp
)
