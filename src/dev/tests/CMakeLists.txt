link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(testTgStructure
    testTgStructure.cpp
) 

add_executable(testTgBuildSpec
    testTgBuildSpec.cpp
) 

add_executable(AppBuilderTest
    AppBuilderTest.cpp
) 

add_executable(AppHilbert2D
    AppHilbert2D.cpp
) 

add_executable(AppConnectorTest
    AppConnectorTest.cpp
) 

add_executable(AppStructureTest
    AppStructureTest.cpp
) 

add_executable(AppRBStringTest
    RBStringTest.cpp
    AppRBString.cpp 
) 

add_executable(AppRotationTest
    AppRotationTest.cpp
) 
