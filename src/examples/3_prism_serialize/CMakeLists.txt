link_directories(${LIB_DIR})

link_libraries(controllers
				tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

	add_executable(AppPrismModel_Serialize
    PrismModel.cpp
    AppPrismModel.cpp
) 

target_link_libraries(AppPrismModel_Serialize ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("config.json" "config.json" COPYONLY)


