# - Try to find MUSIC
# Once done this will define
#  MUSIC_FOUND - System has MUSIC
#  MUSIC_INCLUDE_DIRS - The MUSIC include directories
#  MUSIC_LIBRARIES - The libraries needed to use MUSIC

find_package(PkgConfig)

find_path(MUSIC_INCLUDE_DIR NAMES music.hh)

find_library(MUSIC_LIBRARY NAMES libmusic.so)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MUSIC_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MUSIC DEFAULT_MSG
                                  MUSIC_LIBRARY MUSIC_INCLUDE_DIR)

mark_as_advanced(MUSIC_INCLUDE_DIR MUSIC_LIBRARY )

set(MUSIC_LIBRARIES ${MUSIC_LIBRARY} )
set(MUSIC_INCLUDE_DIRS ${MUSIC_INCLUDE_DIR} )
