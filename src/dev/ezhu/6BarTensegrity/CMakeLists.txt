project(6BarTensegrity)

set(BOOST_HOME_DIR ../../../../env/build/boost/boost)

include_directories(${BOOST_HOME_DIR})
link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})
message(STATUS "BOOST_HOME_DIR: " ${BOOST_HOME_DIR})

link_libraries(core
               tgcreator
               controllers
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	             Adapters
	             TensegrityModel)

#add_executable(App6Bar
#    TensegrityModel.cpp
#    controllers/T6RollingController.cpp
#    models/sixBarModel.cpp
#    App6Bar.cpp)

#add_executable(AppUphillRolling
#    controllers/T6RollingControllerPrism.cpp
#    models/PrismModel_mini.cpp
#    AppPrismModel.cpp)

add_executable(App6Mini
    controllers/T6MiniRollingController.cpp
    models/sixBarMiniModel.cpp
    App6BarMini.cpp)