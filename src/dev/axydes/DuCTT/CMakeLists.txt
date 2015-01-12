link_libraries(tgcreator core Adapters controllers boost_program_options)

set(DuCTT_ROBOT_SRCS
    robot/tgDuCTTHinge.cpp
    robot/tgDuCTTHingeInfo.cpp
    robot/tgPrismatic.cpp
    robot/tgPrismaticInfo.cpp
    robot/tgTouchSensorSphereInfo.cpp
    robot/tgTouchSensorSphereModel.cpp
    robot/DuCTTRobotModel.cpp
)

set(DUCT_SRCS
    ducts/DuctCrossModel.cpp
    ducts/DuctLModel.cpp
    ducts/DuctStraightModel.cpp
    ducts/DuctTeeModel.cpp
)

add_executable(AppDuCTT
    ${DuCTT_ROBOT_SRCS}
    ${DUCT_SRCS}
    controllers/DuCTTSineWaves.cpp
    AppDuCTT.cpp
)

add_executable(AppDuCTTMechTest
    ${DuCTT_ROBOT_SRCS}
    ${DUCT_SRCS}
    controllers/DuCTTMechTestController.cpp
    AppDuCTTMechTest.cpp
)

add_executable(AppDuCTTLearn
    ${DuCTT_ROBOT_SRCS}
    ${DUCT_SRCS}
    controllers/TestLearning.cpp
    AppDuCTTLearn.cpp
)

add_custom_command(TARGET AppDuCTTLearn POST_BUILD
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/controllers/Config.ini ${CMAKE_CURRENT_BINARY_DIR}/Config.ini)

add_custom_command(TARGET AppDuCTTLearn POST_BUILD
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/controllers/logs)
