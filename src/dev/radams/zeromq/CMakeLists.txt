# link_libraries(boost_system boost_chrono)
link_libraries(zmq)

link_directories(${LIB_DIR})

include_directories(${INC_DIR})

add_executable(ZmqHelloWorldServer
    helloWorldServer.cpp
) 

add_executable(ZmqHelloWorldClient
    helloWorldClient.cpp
)

# Weather Update Example
# http://zguide.zeromq.org/page:all#basics
add_executable(wuserver
    wuserver.cpp
)
add_executable(wuclient
    wuclient.cpp
)
 
