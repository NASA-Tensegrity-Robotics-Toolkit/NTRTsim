Project(btietz)

link_libraries(sensors
                tgcreator
                core
                Corde
                util
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
			tgCordeModel.cpp
			tgCordeStringInfo.cpp
            tgRBString.cpp
            tgRBStringInfo.cpp
            )

subdirs(
	Corde
    boxFeet
    caterpillar
    RBTests
    sphereFeet
    TetraSpineStatic
    )
