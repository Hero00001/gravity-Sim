cmake_minimum_required(VERSION 3.16)
project(gravity_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Vendored dependencies (third_party/)
#   - GLM   : header-only math library
#   - GLEW  : prebuilt x64 (headers + glew32.dll + import lib)
#   - GLFW  : prebuilt 3.4 Windows binary (headers + MinGW import lib + dll)
# ---------------------------------------------------------------------------
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# GLM's glm.hpp lives at third_party/glm/glm.hpp, so the include dir is
# the parent (third_party) to resolve <glm/glm.hpp>.
set(GLM_INCLUDE_DIR  "${THIRD_PARTY_DIR}")
set(GLEW_INCLUDE_DIR "${THIRD_PARTY_DIR}/glew/include")
set(GLFW_INCLUDE_DIR "${THIRD_PARTY_DIR}/glfw/include")

set(GLFW_DLL    "${THIRD_PARTY_DIR}/glfw/bin/glfw3.dll")
set(GLEW_DLL    "${THIRD_PARTY_DIR}/glew/bin/glew32.dll")

# Each library ships/needs a different import lib per toolchain:
#   GLFW : glfw3dll.lib (MSVC) vs libglfw3dll.a (MinGW)
#   GLEW : glew32.lib (MSVC, vendored) vs libglew32.a (MinGW, generated
#          from glew32.dll with dlltool)
if(MSVC)
  set(GLFW_IMPLIB "${THIRD_PARTY_DIR}/glfw/lib/glfw3dll.lib")
  set(GLEW_IMPLIB "${THIRD_PARTY_DIR}/glew/lib/glew32.lib")
else()
  set(GLFW_IMPLIB "${THIRD_PARTY_DIR}/glfw/lib/libglfw3dll.a")
  set(GLEW_IMPLIB "${THIRD_PARTY_DIR}/glew/lib/libglew32.a")
endif()

foreach(_f
    "${GLM_INCLUDE_DIR}/glm/glm.hpp"
    "${GLEW_INCLUDE_DIR}/GL/glew.h"
    "${GLFW_INCLUDE_DIR}/GLFW/glfw3.h"
    "${GLFW_IMPLIB}"
    "${GLFW_DLL}"
    "${GLEW_IMPLIB}"
    "${GLEW_DLL}")
  if(NOT EXISTS "${_f}")
    message(FATAL_ERROR "Missing vendored dependency: ${_f}\n"
                        "Restore the third_party/ folder (see the project README).")
  endif()
endforeach()

if(WIN32)
  set(PLATFORM_LIBS opengl32 gdi32 user32 shell32)
else()
  find_package(OpenGL REQUIRED)
  set(PLATFORM_LIBS ${OPENGL_LIBRARIES})
endif()

# Creates an executable with the vendored GLFW/GLEW/GLM and copies the
# runtime DLLs (glfw3.dll, glew32.dll) next to the produced .exe.
function(gravity_target name source)
  add_executable(${name} ${source})
  target_include_directories(${name} PRIVATE
    ${GLM_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR})
  target_link_libraries(${name} PRIVATE
    ${GLFW_IMPLIB}
    ${GLEW_IMPLIB}
    ${PLATFORM_LIBS})
  add_custom_command(TARGET ${name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${GLFW_DLL}" "${GLEW_DLL}"
      "$<TARGET_FILE_DIR:${name}>"
    COMMENT "Copying runtime DLLs for ${name}")
endfunction()

gravity_target(gravity_sim        gravity_sim.cpp)
gravity_target(gravity_sim_3Dgrid gravity_sim_3Dgrid.cpp)
gravity_target(3D_test            3D_test.cpp)
