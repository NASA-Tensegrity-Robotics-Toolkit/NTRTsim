link_directories(${LIB_DIR})

link_libraries(tgcreator core Adapters learningSpines util)

add_executable(ErikAppVerticalSpineCableCollision
    ErikVerticalSpineModelCableCollision.cpp
    ErikAppVerticalSpineCableCollision.cpp
)
