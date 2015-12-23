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

MESSAGE("${PROJECT_NAME}: CMAKE_INSTALL_PREFIX is ${CMAKE_INSTALL_PREFIX}")

# @todo: For some reason this really does not want to work -- maybe because we never to a 'make install' in build.sh?
# install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib COMPONENT libraries)
# install(DIRECTORY ./ DESTINATION include/${PROJECT_NAME}
#        FILES_MATCHING PATTERN "*.h")
        
install(TARGETS ${PROJECT_NAME} 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
