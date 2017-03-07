project(tgcreator)

add_library( ${PROJECT_NAME} SHARED
    tgNodes.cpp
    tgRigidInfo.cpp
    tgRodInfo.cpp
    tgBoxInfo.cpp
    tgBoxMoreAnchorsInfo.cpp
    tgSphereInfo.cpp
    tgStructure.cpp
    tgBuildSpec.cpp
    tgStructureInfo.cpp
    tgConnectorInfo.cpp
    tgCompoundRigidInfo.cpp
    tgPair.cpp
    tgBasicActuatorInfo.cpp
    tgCompressionSpringActuatorInfo.cpp
    tgUnidirComprSprActuatorInfo.cpp
    tgKinematicActuatorInfo.cpp
    tgKinematicContactCableInfo.cpp
    tgBasicContactCableInfo.cpp
    tgRigidAutoCompound.cpp
    tgUtil.cpp
)

link_directories(${LIB_DIR})

# Needed to add boost's random library for tgRigidAutoCompound's hashing function
target_link_libraries(${PROJECT_NAME} core tgOpenGLSupport boost_random)
