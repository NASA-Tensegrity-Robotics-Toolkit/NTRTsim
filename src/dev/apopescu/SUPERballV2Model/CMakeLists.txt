link_libraries(tgcreator controllers core sensors)

add_executable(AppSUPERballV2
    SUPERballV2Model.cpp
    AppSUPERballV2.cpp
    #LengthController.cpp
    RestlenPlaybackController.cpp
)

set(CMAKE_BUILD_TYPE Debug)
