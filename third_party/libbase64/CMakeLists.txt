# Private vendored codec; public API remains vln_agent/utils/base64.hpp.
include(CheckCCompilerFlag)

set(_codecs generic ssse3 sse41 sse42 avx avx2 avx512 neon32 neon64)
set(_sources lib/lib.c lib/codec_choose.c lib/tables/tables.c)
foreach(_codec IN LISTS _codecs)
  list(APPEND _sources lib/arch/${_codec}/codec.c)
endforeach()

# Flags apply only to each SIMD translation unit, never to the caller or dispatcher.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|i.86)$")
  foreach(_codec SSSE3 SSE41 SSE42 AVX AVX2 AVX512)
    if(_codec STREQUAL "SSE41")
      set(_flags "-msse4.1")
    elseif(_codec STREQUAL "SSE42")
      set(_flags "-msse4.2")
    elseif(_codec STREQUAL "AVX512")
      set(_flags "-mavx512vl -mavx512vbmi")
    else()
      string(TOLOWER "${_codec}" _lower)
      set(_flags "-m${_lower}")
    endif()
    check_c_compiler_flag("${_flags}" BASE64_WITH_${_codec})
    if(BASE64_WITH_${_codec})
      string(TOLOWER "${_codec}" _lower)
      set_source_files_properties(lib/arch/${_lower}/codec.c PROPERTIES COMPILE_FLAGS "${_flags}")
    endif()
  endforeach()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(BASE64_WITH_NEON64 ON)
endif()

configure_file(config.h.in config.h @ONLY)
add_library(vln_agent_base64 STATIC ${_sources})
set_target_properties(vln_agent_base64 PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  C_VISIBILITY_PRESET hidden
  C_STANDARD 99
  C_STANDARD_REQUIRED ON)
target_compile_definitions(vln_agent_base64 PUBLIC BASE64_STATIC_DEFINE)
target_include_directories(vln_agent_base64
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
