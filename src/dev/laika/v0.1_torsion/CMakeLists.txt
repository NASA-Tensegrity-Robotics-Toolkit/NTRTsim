link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	       Adapters
	       TensegrityModel)
<<<<<<< HEAD
=======
	  
>>>>>>> 61317e6b5497519355b8febcf46b750f34a6954a

# add_library(TensegrityModel
#     TensegrityModel.cpp)

<<<<<<< HEAD
add_executable(AppHorizontalSpine
  HorizontalSpineController.cpp
  AppHorizontalSpine.cpp
=======
add_executable(AppLaikaTorsion
  LaikaTorsionController.cpp
  AppLaikaTorsion.cpp
>>>>>>> 61317e6b5497519355b8febcf46b750f34a6954a
)


