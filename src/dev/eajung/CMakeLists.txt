link_libraries(tgcreator core Adapters)

#add_executable(AppFlemonsArm
#    FlemonsArm/FlemonsArm.cpp
#    FlemonsArm/AppFlemonsArm.cpp
#)
 
#add_executable(AppScarrArmModel
#    ScarrArm/ScarrArmModel.cpp
#    ScarrArm/AppScarrArm.cpp
#    ScarrArm/ScarrArmController.cpp
#)
#add_executable(AppCrossArm
#    CrossArm/CrossArmModel.cpp
#    CrossArm/AppCrossArm.cpp
#    CrossArm/CrossArmController.cpp
#)

add_executable(AppSaddleArmNew
    SaddleArm/AppSaddleArm.cpp
    SaddleArm/SaddleArmModel.cpp
    SaddleArm/SaddleArmController.cpp
)
 
add_executable(Appv3BallNew
    v3Ball/v3Model.cpp
    v3Ball/Appv3Ball.cpp
) 

add_executable(AppUpperLimbTriangle
    UpperLimbTriangle/AppUpperLimbTriangle.cpp
    UpperLimbTriangle/UpperLimbTriangleModel.cpp
    UpperLimbTriangle/UpperLimbTriangleController.cpp
)

#add_executable(AppSaddleNeckArm
#   SaddleNeckArm/AppSaddleNeckArm.cpp
#   SaddleNeckArm/SaddleNeckArmModel.cpp
#)

