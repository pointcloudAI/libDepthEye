project(Tof_GLFW)

set(CMAKE_CXX_STANDARD 11 )
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)

FIND_PACKAGE(GLFW3)
FIND_PACKAGE(OpenGL)
find_package(OpenCL REQUIRED)

message(STATUS "GLFW3 = ${GLFW3_LIBRARIES} | OPENGL_gl_LIBRARY = ${OPENGL_gl_LIBRARY} | OpenCL_LIBRARIES = ${OpenCL_LIBRARIES}" )

find_package(OpenGL REQUIRED) 
message("OPENGL_gl_LIBRARY = ${OPENGL_gl_LIBRARY}") 
message("OPENGL_LIBRARY = ${OPENGL_LIBRARY}") 
message("GLFW3_LIBRARIES = ${GLFW3_LIBRARIES}") 
message("OPENGL_glu_LIBRARY = ${OPENGL_glu_LIBRARY}")

message(STATUS "GLFW3 = ${GLFW3_LIBRARIES} | OPENGL_gl_LIBRARY = ${OPENGL_gl_LIBRARY}")

INCLUDE_DIRECTORIES(${GLFW3_INCLUDE_DIRS})

include_directories(${OPENGL_INCLUDE_DIR} "/usr/local/include")
  
add_executable(Tof_GLFW Tof_GLFW.cpp)
target_link_libraries(Tof_GLFW  ${OpenCL_LIBRARIES} GPUFrameProcessor pointcloud  ${GLFW3_LIBRARIES} ${OPENGL_gl_LIBRARY}  ${OPENGL_glu_LIBRARY}  -lX11 -lOpenCL )

             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL Tof_GLFW)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library DepthMeasure programs.")
ENDIF()
