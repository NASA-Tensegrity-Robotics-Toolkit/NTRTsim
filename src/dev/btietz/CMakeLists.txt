Project(btietz)

link_libraries(sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
   
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
    hardwareLearning
    hardwareSineWaves
    handTunedSineWaves
    muscleNPTests
    tetraCollisions
    muscleNPEnergy
    TCContact
    )
