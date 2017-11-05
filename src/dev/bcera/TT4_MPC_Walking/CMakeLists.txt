set(BOOST_HOME_DIR ../../../../env/build/boost/boost)

include_directories(${BOOST_HOME_DIR})
link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})
message(STATUS "BOOST_HOME_DIR: " ${BOOST_HOME_DIR})


link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
		controllers
		Adapters
                tgOpenGLSupport)

add_executable(App_TT4_MPC_Walking
    PrismModel_modified.cpp
    AppPrismModel.cpp
    RPThruster.cpp
    Marker.cpp
    controllers/T6RollingControllerPrism.cpp
    controllers/SimulatedSensors.cpp
) 

