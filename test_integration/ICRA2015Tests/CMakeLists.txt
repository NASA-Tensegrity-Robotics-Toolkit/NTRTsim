link_directories(${ENV_LIB_DIR} ${NTRT_BUILD_DIR})

link_libraries(
                tgOpenGLSupport)
             
#add_executable(ICRA2015_test
#  ICRA2015_test.cpp)

#target_link_libraries(ICRA2015_test ${ENV_LIB_DIR}/libgtest.a pthread 
#			${NTRT_BUILD_DIR}/core/libcore.so 
#			${NTRT_BUILD_DIR}/helpers/libFileHelpers.so 
#			${NTRT_BUILD_DIR}/examples/learningSpines/liblearningSpines.so 
#			${NTRT_BUILD_DIR}/dev/btietz/TetraSpineStatic/libtetraSpineHardware.so)

# Temporary test that replaces ICRA2015_test while github issue 191 is worked on.
# This is the same test as ICRA2015_test but uses a model that defines muscle pairs in a
# different order (see github issue 191 for more details).
add_executable(ICRA2015_pair_ordering_test
  ICRA2015_pair_ordering_test.cpp)

target_link_libraries(ICRA2015_pair_ordering_test ${ENV_LIB_DIR}/libgtest.a pthread
     ${NTRT_BUILD_DIR}/core/libcore.so
     ${NTRT_BUILD_DIR}/helpers/libFileHelpers.so
     ${NTRT_BUILD_DIR}/examples/learningSpines/liblearningSpines.so
     ${NTRT_BUILD_DIR}/dev/btietz/TetraSpineStatic/libtetraSpineHardware.so)