link_libraries(tgcreator core controllers)

add_executable(AppDuCTT
    tgRodHinge.cpp
    tgRodHingeInfo.cpp
    tgDuCTTHingeInfo.cpp
    tgPrismatic.cpp
    tgPrismaticInfo.cpp
    DuctStraightModel.cpp
    DuCTTSineWaves.cpp
    DuCTTRobotModel.cpp
    AppDuCTT.cpp
)
