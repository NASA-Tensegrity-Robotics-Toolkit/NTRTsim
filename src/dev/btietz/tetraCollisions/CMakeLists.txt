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
            

configure_file("controlVars.json" "controlVars.json" COPYONLY)

