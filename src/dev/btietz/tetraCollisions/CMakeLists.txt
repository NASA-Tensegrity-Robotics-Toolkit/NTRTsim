link_directories(${LIB_DIR})


link_libraries(obstacles
                tetraSpineLearningSine
				tetraSpineHardware
				learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_library(tetraCollisions
    TetraSpineCollisions.cpp
    colSpineSine.cpp
)
                
add_executable(AppTetraSpineCol
    tscDataLogger.cpp
    tscDataObserver.cpp
	TetraSpineCollisions.cpp
	colSpineSine.cpp
	Wall.cpp
	AppTetraSpineCol.cpp
)

target_link_libraries(AppTetraSpineCol ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

