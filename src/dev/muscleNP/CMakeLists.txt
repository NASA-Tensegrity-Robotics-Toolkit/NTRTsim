Project(MuscleNP)

link_libraries(tgcreator
				core
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
   
            tgGhostInfo.cpp
            tgGhostModel.cpp
            )
            
target_link_libraries(${PROJECT_NAME} tgcreator core tgOpenGLSupport)

