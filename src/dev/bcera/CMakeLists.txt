link_libraries(tgcreator core controllers)

subdirs(
orient
thruster
payload_thruster
payload_thruster_disturbance_rejection
TT3_payload_thruster_w_Walking
V3_V4
OLD_TT3_payload_thruster
TT_Mini_Inclined_Walking
)
	
 add_executable(AppWorkspace
    RPModel2.cpp
    AppRocketPower.cpp
    RPLengthController_Midflight_config1.cpp
    RPThruster_simple.cpp
) 
