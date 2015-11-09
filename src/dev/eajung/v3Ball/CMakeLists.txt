link_libraries(tgcreator controllers core)

add_executable(Appv3BallNew
    v3Model.cpp
    v4Model.cpp
    Appv3Ball.cpp
    v3TensionController.cpp
    v4TensionController.cpp
) 

# To compile a controller, add a line like the
# following inside add_executable:
#    controllers/T6TensionController.cpp
