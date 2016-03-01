#add_library(
#  tgDataObserver.cpp
#  tgDataLogger.cpp
#)

link_libraries(tgcreator core Adapters sensors)

add_executable(AppVerticalSpine
    VerticalSpineModel.cpp
    AppVerticalSpine.cpp
    VerticalSpineBendingController.cpp
)