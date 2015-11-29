link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppTensegrityModel
    TensegrityModel.cpp
    AppTensegrityModel.cpp
) 

target_link_libraries(AppTensegrityModel ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
