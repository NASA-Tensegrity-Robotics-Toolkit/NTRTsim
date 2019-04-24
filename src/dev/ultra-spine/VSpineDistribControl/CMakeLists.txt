#add_library(
#  tgDataObserver.cpp
#  tgDataLogger.cpp
#)

link_libraries(tgcreator core Adapters sensors)

add_executable(AppVSpineDistribControl
    VSpineDistribControlModel.cpp
    AppVSpineDistribControl.cpp
    VSpineDistribControlBending.cpp
    #VSpineDistribControlPassive.cpp
)
