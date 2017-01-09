link_directories(${LIB_DIR})

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

add_library(flemonsSpineContact 
                FlemonsSpineModelContact.cpp
            )
                
add_executable(AppFlemonsSpineContact
    FlemonsSpineModelContact.cpp
    AppFlemonsSpineContact.cpp
    
) 
