# Fails when the library needs a shared library whose name matches one of FORBIDDEN at run time.
#   cmake -DLIB=<shared library> -DFORBIDDEN=<name>,<name> -P needed_libraries.cmake

cmake_minimum_required(VERSION 3.16)

# Commas, because a semicolon does not survive the add_test command line
string(REPLACE "," ";" forbidden "${FORBIDDEN}")

find_program(OBJDUMP objdump)
if(NOT OBJDUMP)
  message(FATAL_ERROR "objdump not found, install binutils")
endif()

execute_process(COMMAND ${OBJDUMP} -p ${LIB}
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "objdump failed on ${LIB}: ${error}")
endif()

string(REGEX MATCHALL "NEEDED +[^\n]+" needed "${output}")
set(found "")
foreach(line IN LISTS needed)
  foreach(name IN LISTS forbidden)
    if(line MATCHES "lib${name}\\.so")
      string(REGEX REPLACE "NEEDED +" "" library "${line}")
      string(APPEND found "  ${library}\n")
    endif()
  endforeach()
endforeach()

if(NOT found STREQUAL "")
  message(FATAL_ERROR "needed at run time, but the library should carry it:\n${found}")
endif()
