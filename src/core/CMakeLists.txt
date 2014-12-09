#Project name must match folder name for includes to work!
project(core)

add_library( ${PROJECT_NAME} SHARED
  tgWorldBulletPhysicsImpl.cpp
    tgBulletSpringCableAnchor.cpp
    Muscle2P.cpp
    ImpedanceControl.cpp
    
    tgModel.cpp
    tgBaseString.cpp
    tgLinearString.cpp
    tgWorld.cpp
    tgSimulation.cpp
    tgBulletRenderer.cpp
    tgSimView.cpp
    tgSimViewGraphics.cpp
    
    tgBulletUtil.cpp
    tgBaseRigid.cpp
    tgRod.cpp
    tgBox.cpp
    tgSphere.cpp
    
    abstractMarker.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} terrain tgOpenGLSupport)

subdirs(
    terrain
)
