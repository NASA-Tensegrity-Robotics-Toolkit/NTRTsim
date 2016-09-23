project(WorldImportTest)

#set(BULLET_HOME_DIR /home/edward/NTRTsim/env/build/bullet)
set(BULLET_HOME_DIR ../../../../env/build/bullet)
set(BULLET_SERIALIZE_DIR ${BULLET_HOME_DIR}/Extras/Serialize)

message(STATUS "BULLET_HOME_DIR: " ${BULLET_HOME_DIR})
message(STATUS "BULLET_SERIALIZE_DIR: " ${BULLET_SERIALIZE_DIR})

include_directories(
${BULLET_SERIALIZE_DIR}/BulletFileLoader
${BULLET_SERIALIZE_DIR}/BulletWorldImporter
)

link_directories(
${LIB_DIR}
${BULLET_SERIALIZE_DIR}/BulletFileLoader
${BULLET_SERIALIZE_DIR}/BulletWorldImporter
)

link_libraries(core
               tgcreator
               controllers
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	       Adapters
	       TensegrityModel
               BulletFileLoader
               BulletWorldImporter)

# add_library(TensegrityModel
#     TensegrityModel.cpp)

add_executable(AppWorldImportTest
#    TensegrityModel.cpp
    RPLengthController.h
    RPLengthController_rand.cpp
    AppWorldImportTest.cpp)


