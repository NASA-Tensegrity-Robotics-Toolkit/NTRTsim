Project(Aliakbar)

link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppAATModel
    AATModel.cpp
    AppAATModel.cpp
    tgBallJoint.cpp
    tgBallJointInfo.cpp
    AATController.cpp
    
) 

