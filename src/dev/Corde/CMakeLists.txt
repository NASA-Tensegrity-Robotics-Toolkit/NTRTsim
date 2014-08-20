Project(Corde)

add_library(${PROJECT_NAME} SHARED
			   cordeAnchor.cpp
			   cordeCollisionObject.cpp
			   cordeCollisionShape.cpp
			   #cordeConcaveCollisionAlgorithm.cpp
			   #cordeDynamicsWorld.cpp
			   CordeModel.cpp
			   cordeRigidBodyCollisionConfiguration.cpp
			   cordeRigidCollisionAlgorithm.cpp
			   cordeSelfCollisionAlgorithm.cpp
			   defaultCordeSolver.cpp
            )

