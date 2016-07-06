link_libraries(tgcreator core controllers)

subdirs(
orient
thruster
payload_thruster
payload_thruster_disturbance_rejection
TT3_payload_thruster
V3_V4
)
	
 add_executable(AppWorkspace
    RPModel2.cpp
    AppRocketPower.cpp
    RPLengthController_Midflight_config1.cpp
    RPThruster_simple.cpp
) 
