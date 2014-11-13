link_libraries(tgcreator core)

add_executable(AppHinge
    ../DuCTT/tgRodHinge.cpp
    ../DuCTT/tgRodHingeInfo.cpp
    ../DuCTT/tgTouchSensorSphereInfo.cpp
    ../DuCTT/tgTouchSensorSphereModel.cpp
    HingeModel.cpp
    AppHinge.cpp
)
