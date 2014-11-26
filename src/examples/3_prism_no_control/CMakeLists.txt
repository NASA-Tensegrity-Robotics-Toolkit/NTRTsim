link_directories(${LIB_DIR})

link_libraries(controllers
				tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppPrismModel_nc
    PrismModel.cpp
    AppPrismModel.cpp
) 

