project(DepthMeasure)

set(CMAKE_CXX_STANDARD 11)

find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)

FIND_PACKAGE(GLFW3)
FIND_PACKAGE(OpenGL)

message(STATUS "GLFW3 = ${GLFW3_LIBRARIES} | OPENGL_gl_LIBRARY = ${OPENGL_gl_LIBRARY}")

find_package(OpenGL REQUIRED) 
message("OPENGL_gl_LIBRARY = ${OPENGL_gl_LIBRARY}") 
message("OPENGL_LIBRARY = ${OPENGL_LIBRARY}") 
message("GLFW3_LIBRARIES = ${GLFW3_LIBRARIES}") 
message("OPENGL_glu_LIBRARY = ${OPENGL_glu_LIBRARY}")

message(STATUS "GLFW3 = ${GLFW3_LIBRARIES} | OPENGL_gl_LIBRARY = ${OPENGL_gl_LIBRARY}")

INCLUDE_DIRECTORIES(${GLFW3_INCLUDE_DIRS})

include_directories(${OPENGL_INCLUDE_DIR} "/usr/local/include")
  
add_executable(Tof_GLFW Tof_GLFW.cpp)
target_link_libraries(Tof_GLFW pointcloud ${GLFW3_LIBRARIES} ${OPENGL_gl_LIBRARY}  ${OPENGL_glu_LIBRARY})

             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL Tof_GLFW)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library DepthMeasure programs.")
ENDIF()
