link_libraries(tgcreator core controllers)

subdirs(
orient
thruster
payload_thruster
TT3_payload_thruster
)
	
 add_executable(AppWorkspace
    RPModel2.cpp
    AppRocketPower.cpp
    RPLengthController_Midflight_config1.cpp
    RPThruster_simple.cpp
) 
