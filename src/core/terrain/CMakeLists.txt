project(terrain)

add_library( ${PROJECT_NAME} SHARED
tgBulletGround.cpp
tgBoxGround.cpp
tgPlaneGround.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME})
