link_directories(${LIB_DIR})

link_libraries(controllers
				tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppPrismModel
    PrismModel.cpp
    PrismTensionController.cpp
    AppPrismModel.cpp
) 

