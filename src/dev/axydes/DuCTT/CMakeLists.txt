link_libraries(tgcreator core controllers boost_program_options)

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
    ducts/DuctCrossModel.cpp
    ducts/DuctLModel.cpp
    ducts/DuctStraightModel.cpp
    ducts/DuctTeeModel.cpp
    AppDuCTT.cpp
)
