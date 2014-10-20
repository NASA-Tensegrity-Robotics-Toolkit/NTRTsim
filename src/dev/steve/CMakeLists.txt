link_libraries(tgcreator core Adapters)

add_executable(AppT12SuperBallPayload
    T12SuperBallPayload/T12SuperBallPayload.cpp
    T12SuperBallPayload/T12SuperBallPayloadController.cpp
    T12SuperBallPayload/AppT12SuperBallPayload.cpp
)

file(COPY Escape_T6/Config.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(AppEscape_T6
    Escape_T6/Crater.cpp
    Escape_T6/CraterDeep.cpp
    Escape_T6/Escape_T6Model.cpp
    Escape_T6/Escape_T6Controller.cpp
    Escape_T6/AppEscape_T6.cpp
) 

add_custom_command(TARGET AppEscape_T6 POST_BUILD
					COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/Escape_T6/logs)
