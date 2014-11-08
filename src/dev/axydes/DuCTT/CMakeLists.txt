link_libraries(tgcreator core controllers boost_program_options)

add_executable(AppDuCTT
    tgRodHinge.cpp
    tgRodHingeInfo.cpp
    tgDuCTTHingeInfo.cpp
    tgPrismatic.cpp
    tgPrismaticInfo.cpp
    tgTouchSensorSphereInfo.cpp
    tgTouchSensorSphereModel.cpp
    DuctStraightModel.cpp
    DuCTTSineWaves.cpp
    DuCTTRobotModel.cpp
    AppDuCTT.cpp
)
