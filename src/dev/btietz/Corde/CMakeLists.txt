link_directories(${LIB_DIR})

link_libraries(btietz
				tgcreator
                util
                sensors
                core
                Corde    
                terrain 
                tgOpenGLSupport)


add_executable(AppCordeTest
    NestedStructureTestModel.cpp
    T6Model.cpp
    simpleCordeTensegrity.cpp
    AppCordeTest.cpp
)

add_executable(AppVectorTest
	AppVectorTest.cpp
)

add_executable(AppVectorTestaa
	AppVectorTestaa.cpp
)
