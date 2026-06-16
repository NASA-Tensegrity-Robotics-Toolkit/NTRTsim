cmake_minimum_required(VERSION 2.6)

PROJECT(NASA_Tensegrity_Robotics_Toolkit)

add_definitions(-DBOOST_NO_CXX11_HDR_TUPLE -DBOOST_NO_CXX11_HDR_ARRAY -DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

# Add your subdirectories here
subdirs(
    core
    helpers
    learning
    util
    sensors
    tgcreator
    models
    controllers
    dev
    examples
    yamlbuilder
)

# To turn off verbose compiling, comment out
# the -Wall below.
#add_definitions(
#    -Wall
#)

# Don't need to edit below this line.

# Set some convenience variables

SET(ENV_DIR ${PROJECT_SOURCE_DIR}/../env)
SET(ENV_INC_DIR ${ENV_DIR}/include)
SET(ENV_LIB_DIR ${ENV_DIR}/lib)

# Bring in the bullet graphics dependencies, etc. 

include(inc.CMakeBullet.txt)


