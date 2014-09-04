project(tgcreator)

add_library( ${PROJECT_NAME} SHARED
    tgNodes.cpp
    tgRigidInfo.cpp
    tgRodInfo.cpp
    tgBoxInfo.cpp
    tgStructure.cpp
    tgBuildSpec.cpp
    tgStructureInfo.cpp
    tgConnectorInfo.cpp
    tgCompoundRigidInfo.cpp
    tgPair.cpp
    tgLinearStringInfo.cpp
    tgRigidAutoCompound.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} core util tgOpenGLSupport)
