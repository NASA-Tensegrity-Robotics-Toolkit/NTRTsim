link_libraries(learningSpines
                sensors
                tgcreator             
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_executable(App12Bar
    App12BarCpp.cpp
    T12Model.cpp
    T12Controller.cpp
) 