project(terrain)

add_library( ${PROJECT_NAME} SHARED
tgBulletGround.cpp
tgBoxGround.cpp
tgEmptyGround.cpp
tgPlaneGround.cpp
tgCraterGround.cpp
tgHillyGround.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME})
