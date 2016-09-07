link_directories( ${LIB_DIR} )

link_libraries(tgcreator
  controllers
  core
  forceplate)

# @to-do: what needs to be compiled in to use another tgModel?
add_executable(AppForcePlateDemo
  #  ForcePlateModel.cpp
  ../3_prism/PrismModel.cpp
  AppForcePlateDemo.cpp
) 
