# Fails when the library exports a symbol outside the public API, or a line of the public API
# matches no exported symbol.
#   cmake -DLIB=<shared library> -DALLOWED=<prefix file> -P exported_symbols.cmake
# A symbol passes when its demangled name starts with a line of the prefix file, or when it is in
# namespace std, which libstdc++ declares with default visibility.

cmake_minimum_required(VERSION 3.16)

find_program(NM nm)
if(NOT NM)
  message(FATAL_ERROR "nm not found, install binutils")
endif()

# Unsorted, so line i of one run is line i of the other
execute_process(COMMAND ${NM} -D -p --defined-only ${LIB}
  RESULT_VARIABLE result OUTPUT_VARIABLE mangled_output ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "nm failed on ${LIB}: ${error}")
endif()
execute_process(COMMAND ${NM} -D -p --defined-only -C ${LIB}
  OUTPUT_VARIABLE demangled_output)

string(REGEX REPLACE "\n$" "" mangled_output "${mangled_output}")
string(REGEX REPLACE "\n$" "" demangled_output "${demangled_output}")
string(REPLACE "\n" ";" mangled_lines "${mangled_output}")
string(REPLACE "\n" ";" demangled_lines "${demangled_output}")
file(STRINGS ${ALLOWED} prefixes REGEX "^[^#]")

set(unexpected "")
set(matched "")
list(LENGTH mangled_lines count)
math(EXPR last "${count} - 1")
foreach(i RANGE ${last})
  list(GET mangled_lines ${i} mangled)
  list(GET demangled_lines ${i} demangled)
  # <address> <type> <name>
  string(REGEX REPLACE "^[0-9a-f]* [A-Za-z] " "" mangled "${mangled}")
  string(REGEX REPLACE "^[0-9a-f]* [A-Za-z] " "" demangled "${demangled}")

  # std and its abbreviations (std::allocator, std::string, ...), also as typeinfo, vtable and
  # function-local statics
  if(mangled MATCHES "^_Z(T[ISV]|Z|GVZ)?(N[rVK]*)?S[tabsiod]")
    continue()
  endif()

  set(allowed FALSE)
  foreach(prefix IN LISTS prefixes)
    string(FIND "${demangled}" "${prefix}" position)
    if(position EQUAL 0)
      set(allowed TRUE)
      list(APPEND matched "${prefix}")
      break()
    endif()
  endforeach()
  if(NOT allowed)
    string(APPEND unexpected "  ${demangled}\n")
  endif()
endforeach()

# A public declaration without AIDIN_HAND2_EXPORT exports nothing
set(missing "")
foreach(prefix IN LISTS prefixes)
  if(NOT prefix IN_LIST matched)
    string(APPEND missing "  ${prefix}\n")
  endif()
endforeach()

if(NOT unexpected STREQUAL "" OR NOT missing STREQUAL "")
  message(FATAL_ERROR
    "exported outside the public API:\n${unexpected}"
    "public API not exported:\n${missing}")
endif()
