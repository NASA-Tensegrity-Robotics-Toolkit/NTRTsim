link_libraries(tgcreator controllers core sensors)

add_executable(ApptestSUPERball
    T6Model.cpp
    AppSUPERball.cpp
    T6TensionController.cpp
) 

# To compile a controller, add a line like the
# following inside add_executable:
#    controllers/T6TensionController.cpp
