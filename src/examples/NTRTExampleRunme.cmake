# Generate a runme launcher script in the build tree for an example executable.
#
# Usage:
#   ntrt_add_runme(MyTarget)
#   ntrt_add_runme(MyTarget SCRIPT_NAME runme-other)
function(ntrt_add_runme target)
  set(options)
  set(oneValueArgs SCRIPT_NAME DESCRIPTION)
  cmake_parse_arguments(NTRT "" "SCRIPT_NAME;DESCRIPTION" "" ${ARGN})
  if(NOT NTRT_SCRIPT_NAME)
    set(NTRT_SCRIPT_NAME "runme")
  endif()
  if(NOT NTRT_DESCRIPTION)
    set(NTRT_DESCRIPTION "Run ${target}")
  endif()
  set(APP_NAME "${target}")
  set(APP_DESCRIPTION "${NTRT_DESCRIPTION}")
  configure_file(
    "${CMAKE_SOURCE_DIR}/examples/runme.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${NTRT_SCRIPT_NAME}"
    @ONLY
  )
  add_custom_command(
    TARGET ${target} POST_BUILD
    COMMAND chmod +x "${CMAKE_CURRENT_BINARY_DIR}/${NTRT_SCRIPT_NAME}"
    COMMENT "Making ${NTRT_SCRIPT_NAME} executable"
  )
endfunction()
