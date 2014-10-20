Project(MuscleNP)

link_libraries(core
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
   
            MuscleNP.cpp
            tgMultiPointStringInfo.cpp
            )

