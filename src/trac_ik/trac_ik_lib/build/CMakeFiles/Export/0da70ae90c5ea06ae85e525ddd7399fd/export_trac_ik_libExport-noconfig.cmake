#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "trac_ik_lib::trac_ik_lib" for configuration ""
set_property(TARGET trac_ik_lib::trac_ik_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(trac_ik_lib::trac_ik_lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtrac_ik_lib.so"
  IMPORTED_SONAME_NOCONFIG "libtrac_ik_lib.so"
  )

list(APPEND _cmake_import_check_targets trac_ik_lib::trac_ik_lib )
list(APPEND _cmake_import_check_files_for_trac_ik_lib::trac_ik_lib "${_IMPORT_PREFIX}/lib/libtrac_ik_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
