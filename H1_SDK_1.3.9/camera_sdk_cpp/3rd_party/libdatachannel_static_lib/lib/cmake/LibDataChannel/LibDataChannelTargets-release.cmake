#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LibDataChannel::usrsctp" for configuration "Release"
set_property(TARGET LibDataChannel::usrsctp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LibDataChannel::usrsctp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libusrsctp.a"
  )

list(APPEND _cmake_import_check_targets LibDataChannel::usrsctp )
list(APPEND _cmake_import_check_files_for_LibDataChannel::usrsctp "${_IMPORT_PREFIX}/lib/libusrsctp.a" )

# Import target "LibDataChannel::srtp2" for configuration "Release"
set_property(TARGET LibDataChannel::srtp2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LibDataChannel::srtp2 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsrtp2.a"
  )

list(APPEND _cmake_import_check_targets LibDataChannel::srtp2 )
list(APPEND _cmake_import_check_files_for_LibDataChannel::srtp2 "${_IMPORT_PREFIX}/lib/libsrtp2.a" )

# Import target "LibDataChannel::LibJuice" for configuration "Release"
set_property(TARGET LibDataChannel::LibJuice APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LibDataChannel::LibJuice PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libjuice.a"
  )

list(APPEND _cmake_import_check_targets LibDataChannel::LibJuice )
list(APPEND _cmake_import_check_files_for_LibDataChannel::LibJuice "${_IMPORT_PREFIX}/lib/libjuice.a" )

# Import target "LibDataChannel::LibDataChannel" for configuration "Release"
set_property(TARGET LibDataChannel::LibDataChannel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LibDataChannel::LibDataChannel PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdatachannel.a"
  )

list(APPEND _cmake_import_check_targets LibDataChannel::LibDataChannel )
list(APPEND _cmake_import_check_files_for_LibDataChannel::LibDataChannel "${_IMPORT_PREFIX}/lib/libdatachannel.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
