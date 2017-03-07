link_libraries(obstacles
                tetraSpineLearningSine
                tetraSpineHardware
                learningSpines
                sensors
                tgcreator
                core
                terrain
                tgOpenGLSupport)

add_library(tetraCollisions SHARED
    TetraSpineCollisions.cpp
    colSpineSine.cpp
)

add_library(ContactCableCons SHARED
			ContactCableDemo.cpp)

add_executable(AppTetraSpineCol
    TetraSpineCollisions.cpp
    colSpineSine.cpp
    AppTetraSpineCol.cpp
)
			
add_executable(AppContactCables
    ContactCableDemo.cpp
    AppContactCables.cpp
) 
