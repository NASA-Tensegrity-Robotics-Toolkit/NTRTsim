link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(tgcreator
		Adapters
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport
		yaml-cpp
		TensegrityModel)

add_executable(AppEscapeML
    Crater.cpp
    CraterDeep.cpp
    EscapeMLController.cpp
    EscapeMLModel.cpp
    AppEscapeML.cpp
) 