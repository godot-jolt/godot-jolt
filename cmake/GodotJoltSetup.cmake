include_guard()

include(GodotJoltConfigs)
include(GodotJoltOptions)

# Set EP_BASE so we get a nicer directory structure for external projects
set_directory_properties(PROPERTIES EP_BASE ${PROJECT_BINARY_DIR}/External)

if(PROJECT_IS_TOP_LEVEL)
	# Group targets into folders for any IDE that supports it (like Visual Studio)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)

	# Ensure we output to the `bin` directory when building from Visual Studio
	set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD TRUE)
endif()

macro(is_targeting instruction_sets output_variable)
	if(GDJOLT_X86_INSTRUCTION_SET MATCHES "^(${instruction_sets})$")
		set(${output_variable} TRUE)
	else()
		set(${output_variable} FALSE)
	endif()
endmacro()

is_targeting(AVX512 GDJOLT_USE_AVX512)
is_targeting(AVX512|AVX2 GDJOLT_USE_AVX2)
is_targeting(AVX512|AVX2 GDJOLT_USE_BMI1)
is_targeting(AVX512|AVX2 GDJOLT_USE_FMA3)
is_targeting(AVX512|AVX2 GDJOLT_USE_F16C)
is_targeting(AVX512|AVX2|AVX GDJOLT_USE_AVX)
is_targeting(AVX512|AVX2|AVX GDJOLT_USE_SSE4_2)
is_targeting(AVX512|AVX2|AVX|SSE2 GDJOLT_USE_SSE2)

if(GDJOLT_CROSS_PLATFORM_DETERMINISTIC)
	set(GDJOLT_USE_FMA3 FALSE)
endif()
