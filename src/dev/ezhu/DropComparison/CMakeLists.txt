link_libraries(tgcreator controllers core)

add_executable(AppDropComparison
    v3Model.cpp
    v4Model.cpp
    AppDropComparison.cpp
    v3TensionController.cpp
    v4TensionController.cpp
) 
