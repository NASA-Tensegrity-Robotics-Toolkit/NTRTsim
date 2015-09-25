link_libraries(tgcreator controllers core)

add_executable(AppSUPERball
    T6Model.cpp
    AppSUPERball.cpp
    controllers/T6zeroMQController.cpp
) 

# To compile a controller, add a line like the
# following inside add_executable:
# controllers/T6TensionController.cpp
