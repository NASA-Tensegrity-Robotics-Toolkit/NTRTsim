project(tgcreator)

add_library( ${PROJECT_NAME} SHARED
    tgNodes.cpp
    tgRigidInfo.cpp
    tgRodInfo.cpp
    tgBoxInfo.cpp
    tgSphereInfo.cpp
    tgStructure.cpp
    tgBuildSpec.cpp
    tgStructureInfo.cpp
    tgConnectorInfo.cpp
    tgCompoundRigidInfo.cpp
    tgPair.cpp
    tgBasicActuatorInfo.cpp
    tgKinematicActuatorInfo.cpp
    tgKinematicContactCableInfo.cpp
    tgBasicContactCableInfo.cpp
    tgRigidAutoCompound.cpp
    tgUtil.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} core tgOpenGLSupport)
