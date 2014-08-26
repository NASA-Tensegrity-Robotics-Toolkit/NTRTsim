option(BUILD_DUCTT "Build the DuCTT robot model" OFF)

if(BUILD_DUCTT)

    link_libraries(tgcreator core)

    add_executable(AppDuCTTTest
        tgPrismatic.cpp
        tgPrismaticInfo.cpp
        DuCTTTestModel.cpp
        AppDuCTT.cpp
    )

endif()
