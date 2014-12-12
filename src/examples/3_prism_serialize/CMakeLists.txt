link_directories(${LIB_DIR})

link_libraries(tgcreator
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
