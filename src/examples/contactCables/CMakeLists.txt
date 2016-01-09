link_libraries(tgcreator core)

add_library(ContactCableCons SHARED
			ContactCableDemo.cpp)

add_executable(AppContactCables
    ContactCableDemo.cpp
    AppContactCables.cpp
) 
