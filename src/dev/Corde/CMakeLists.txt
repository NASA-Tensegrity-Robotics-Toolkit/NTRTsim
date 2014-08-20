Project(Corde)

add_library(${PROJECT_NAME} SHARED
			   cordeAnchor.cpp
			   cordeCollisionObject.cpp
			   cordeCollisionShape.cpp
			   CordeModel.cpp
			   defaultCordeSolver.cpp
            )

