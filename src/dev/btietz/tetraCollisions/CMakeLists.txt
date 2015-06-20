link_directories(${LIB_DIR})


link_libraries(obstacles
                tetraSpineLearningSine
				tetraSpineHardware
				learningSpines
                sensors
                tgcreator
                core
                terrain
                tgOpenGLSupport)

add_library(tetraCollisions SHARED
    tscDataLogger.cpp
    tscDataObserver.cpp
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
target_link_libraries(tetraCollisions ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

