IF(PKG_CONFIG_FOUND)
  IF(APPLE)
    # homebrew or macports pkgconfig locations
    SET(ENV{PKG_CONFIG_PATH} "/usr/local/opt/jpeg-turbo/lib/pkgconfig:/opt/local/lib/pkgconfig")
  ENDIF()
  SET(ENV{PKG_CONFIG_PATH} "${DEPENDS_DIR}/jpeg-turbo/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
  PKG_CHECK_MODULES(TURBOJPEG libturbojpeg)

  FIND_LIBRARY(TURBOJPEG_LIBRARY
    NAMES ${TURBOJPEG_LIBRARIES}
    HINTS ${TURBOJPEG_LIBRARY_DIRS}
  )
  SET(TURBOJPEG_LIBRARIES ${TURBOJPEG_LIBRARY})
  MESSAGE(STATUS "TURBOJPEG = ${TURBOJPEG_LIBRARIES} include path= ${TURBOJPEG_INCLUDE_DIRS}")
  RETURN()
ENDIF()


