include_guard()

if(NOT APPLE)
	set(GDJOLT_STATIC_RUNTIME_LIBRARY TRUE
		CACHE BOOL
		"Use static C++ runtime library."
	)
else()
	set(GDJOLT_STATIC_RUNTIME_LIBRARY FALSE)
endif()

set(GDJOLT_CROSS_PLATFORM_DETERMINISTIC FALSE
	CACHE BOOL
	"Compile in such a way as to attempt to keep things deterministic across platforms."
)

set(GDJOLT_LTO TRUE
	CACHE BOOL
	"Enable link-time optimizations for any optimized builds."
)

set(GDJOLT_PRECOMPILE_HEADERS TRUE
	CACHE BOOL
	"Precompile header files that don't change often, like external ones."
)

if(NOT APPLE)
	set(GDJOLT_X86_INSTRUCTION_SET SSE2
		CACHE STRING
		"Minimum required CPU instruction set when compiling for x86."
	)

	set(x86_instruction_sets NONE SSE2 AVX AVX2 AVX512)
	set_property(CACHE GDJOLT_X86_INSTRUCTION_SET PROPERTY STRINGS ${x86_instruction_sets})

	if(NOT GDJOLT_X86_INSTRUCTION_SET IN_LIST x86_instruction_sets)
		message(FATAL_ERROR "Unsupported CPU instruction set: '${GDJOLT_X86_INSTRUCTION_SET}'.")
	endif()
else()
	set(GDJOLT_X86_INSTRUCTION_SET NONE)
endif()
