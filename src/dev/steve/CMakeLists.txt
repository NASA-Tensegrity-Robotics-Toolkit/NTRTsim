link_libraries(tgcreator core Adapters)

add_executable(AppT12SuperBallPayload
    T12SuperBallPayload/T12SuperBallPayload.cpp
    T12SuperBallPayload/T12SuperBallPayloadController.cpp
    T12SuperBallPayload/AppT12SuperBallPayload.cpp
)

add_executable(AppEscape_T6
    Escape_T6/Escape_T6Model.cpp
    Escape_T6/Escape_T6Controller.cpp
    Escape_T6/AppEscape_T6.cpp
) 
