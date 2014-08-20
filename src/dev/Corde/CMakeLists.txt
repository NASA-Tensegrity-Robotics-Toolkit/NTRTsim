Project(Corde)

add_library(${PROJECT_NAME} SHARED
			   cordeAnchor.cpp
			   cordeCollisionShape.cpp
			   cordeCollisionObject.cpp
			   #cordeConcaveCollisionAlgorithm.cpp
			   cordeDynamicsWorld.cpp
			   CordeModel.cpp
			   cordeRigidBodyCollisionConfiguration.cpp
			   cordeRigidCollisionAlgorithm.cpp
			   cordeSelfCollisionAlgorithm.cpp
			   defaultCordeSolver.cpp
            )

