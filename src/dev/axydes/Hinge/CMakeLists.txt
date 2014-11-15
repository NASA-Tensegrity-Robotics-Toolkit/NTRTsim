link_libraries(tgcreator core)

add_executable(AppHinge
    ../DuCTT/robot/tgRodHinge.cpp
    ../DuCTT/robot/tgRodHingeInfo.cpp
    ../DuCTT/robot/tgTouchSensorSphereInfo.cpp
    ../DuCTT/robot/tgTouchSensorSphereModel.cpp
    HingeModel.cpp
    AppHinge.cpp
)
