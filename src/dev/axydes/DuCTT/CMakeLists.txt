link_libraries(tgcreator core Adapters controllers boost_program_options)

file(COPY controllers/Config.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(AppDuCTT
    robot/tgRodHinge.cpp
    robot/tgRodHingeInfo.cpp
    robot/tgDuCTTHingeInfo.cpp
    robot/tgPrismatic.cpp
    robot/tgPrismaticInfo.cpp
    robot/tgTouchSensorSphereInfo.cpp
    robot/tgTouchSensorSphereModel.cpp
    robot/DuCTTRobotModel.cpp
    controllers/DuCTTSineWaves.cpp
    controllers/TestLearning.cpp
    ducts/DuctCrossModel.cpp
    ducts/DuctLModel.cpp
    ducts/DuctStraightModel.cpp
    ducts/DuctTeeModel.cpp
    AppDuCTT.cpp
)

add_custom_command(TARGET AppDuCTT POST_BUILD
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/controllers/logs)
