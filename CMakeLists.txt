cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

include(GodotJoltPrelude)

project(godot-jolt
	VERSION 0.1.0
	LANGUAGES CXX
)

include(GodotJoltSetup)
include(GodotJoltExternalGodotCpp)
include(GodotJoltExternalJolt)
include(GodotJoltExternalMimalloc)

set(source_dir ${CMAKE_CURRENT_LIST_DIR}/src)
set(pch_file ${source_dir}/precompiled.hpp)

file(GLOB_RECURSE godot-jolt_SOURCES CONFIGURE_DEPENDS ${source_dir}/*.cpp)
file(GLOB_RECURSE godot-jolt_HEADERS CONFIGURE_DEPENDS ${source_dir}/*.hpp)

add_library(godot-jolt SHARED ${godot-jolt_SOURCES} ${godot-jolt_HEADERS})
add_library(godot-jolt::godot-jolt ALIAS godot-jolt)

target_link_libraries(godot-jolt
	godot-jolt::godot-cpp
	godot-jolt::jolt
	godot-jolt::mimalloc
)

set(is_windows $<STREQUAL:${CMAKE_SYSTEM_NAME},Windows>)
set(is_linux $<STREQUAL:${CMAKE_SYSTEM_NAME},Linux>)
set(is_macos $<STREQUAL:${CMAKE_SYSTEM_NAME},Darwin>)

set(is_x86 $<STREQUAL:${GDJ_TARGET_ARCHITECTURES},x86>)
set(is_x64 $<STREQUAL:${GDJ_TARGET_ARCHITECTURES},x64>)

set(is_editor_config $<CONFIG:EditorDebug,EditorDevelopment,EditorDistribution>)
set(is_debug_config $<CONFIG:Debug,EditorDebug>)
set(is_development_config $<CONFIG:Development,EditorDevelopment>)
set(is_distribution_config $<CONFIG:Distribution,EditorDistribution>)
set(is_optimized_config $<OR:${is_development_config},${is_distribution_config}>)

string(CONCAT suffix_platform
	$<${is_windows}:_windows>
	$<${is_linux}:_linux>
	$<${is_macos}:_macos>
)

string(CONCAT suffix_arch
	$<${is_x86}:-x86>
	$<${is_x64}:-x64>
)

set(suffix_editor $<${is_editor_config}:_editor>)
set(suffix ${suffix_platform}${suffix_arch}${suffix_editor})

set(use_static_crt $<BOOL:${GDJ_STATIC_RUNTIME_LIBRARY}>)
set(msvcrt_debug $<$<CONFIG:Debug,EditorDebug>:Debug>)
set(msvcrt_dll $<$<NOT:${use_static_crt}>:DLL>)
set(msvcrt MultiThreaded${msvcrt_debug}${msvcrt_dll})

set(target_avx512 $<STREQUAL:${GDJ_X86_INSTRUCTION_SET},AVX512>)
set(target_avx2 $<STREQUAL:${GDJ_X86_INSTRUCTION_SET},AVX2>)
set(target_avx $<STREQUAL:${GDJ_X86_INSTRUCTION_SET},AVX>)
set(target_sse2 $<STREQUAL:${GDJ_X86_INSTRUCTION_SET},SSE2>)

set(is_msvc_like $<BOOL:${MSVC}>)
set(is_msvc_cl $<CXX_COMPILER_ID:MSVC>)
set(is_llvm_clang $<CXX_COMPILER_ID:Clang>)
set(is_gcc $<CXX_COMPILER_ID:GNU>)
set(is_clang_cl $<AND:${is_msvc_like},${is_llvm_clang}>)

set(macro_prefix_option -fmacro-prefix-map=${source_dir}=.)

set_target_properties(godot-jolt PROPERTIES
	OUTPUT_NAME godot-jolt${suffix}
	PREFIX ""
	INCLUDE_DIRECTORIES ${source_dir}
	MSVC_RUNTIME_LIBRARY ${msvcrt}
	CXX_VISIBILITY_PRESET hidden
	INTERPROCEDURAL_OPTIMIZATION_DEVELOPMENT ${GDJ_INTERPROCEDURAL_OPTIMIZATION}
	INTERPROCEDURAL_OPTIMIZATION_DISTRIBUTION ${GDJ_INTERPROCEDURAL_OPTIMIZATION}
	INTERPROCEDURAL_OPTIMIZATION_EDITORDEVELOPMENT ${GDJ_INTERPROCEDURAL_OPTIMIZATION}
	INTERPROCEDURAL_OPTIMIZATION_EDITORDISTRIBUTION ${GDJ_INTERPROCEDURAL_OPTIMIZATION}
	FRAMEWORK TRUE
	MACOSX_FRAMEWORK_IDENTIFIER "org.godotjolt.\${PRODUCT_NAME:rfc1034identifier}"
	MACOSX_FRAMEWORK_BUNDLE_VERSION ${CMAKE_PROJECT_VERSION}
	MACOSX_FRAMEWORK_SHORT_VERSION_STRING ${CMAKE_PROJECT_VERSION}
	MACOSX_FRAMEWORK_ICON_FILE ""
)

target_compile_features(godot-jolt
	PRIVATE cxx_std_17
)

target_compile_definitions(godot-jolt
	PRIVATE $<${is_windows}:GDJ_PLATFORM_WINDOWS>
	PRIVATE $<${is_linux}:GDJ_PLATFORM_LINUX>
	PRIVATE $<${is_macos}:GDJ_PLATFORM_MACOS>
	PRIVATE $<${is_debug_config}:GDJ_CONFIG_DEBUG>
	PRIVATE $<${is_development_config}:GDJ_CONFIG_DEVELOPMENT>
	PRIVATE $<${is_distribution_config}:GDJ_CONFIG_DISTRIBUTION>
	PRIVATE $<${is_editor_config}:GDJ_CONFIG_EDITOR>
	PRIVATE $<IF:${is_debug_config},_DEBUG,NDEBUG>
)

if(GDJ_PRECOMPILE_HEADERS)
	target_precompile_headers(godot-jolt
		PRIVATE ${pch_file}
	)
else()
	target_compile_options(godot-jolt
		PRIVATE $<IF:${is_msvc_like},/FI${pch_file},-include${pch_file}>
	)
endif()

if(MSVC)
	set(pdb_file $<TARGET_PDB_FILE:godot-jolt>)
	set(pdb_file_name $<TARGET_PDB_FILE_NAME:godot-jolt>)

	target_compile_options(godot-jolt
		PRIVATE /utf-8 # Treat lack of BOM as UTF-8
		PRIVATE /W4 # Warning level 4
		PRIVATE /w44245 # Enable implicit conversion warning
		PRIVATE /w44365 # Enable another implicit conversion warning
		PRIVATE /w44800 # Enable another implicit conversion warning
		PRIVATE /wd4324 # Disable structure padding warning
		PRIVATE /EHsc # Enable exception handling
		PRIVATE /GF # Enable string pooling
		PRIVATE /Gy # Enable function-level linking
		PRIVATE /permissive- # Enable standard conformance
		PRIVATE /Zc:__cplusplus # Enable updated `__cplusplus` macro
		PRIVATE /Zc:inline # Remove unreferenced COMDAT
		PRIVATE /Zc:lambda # Enable updated lambda processor
		PRIVATE /Zc:preprocessor # Enable standard-conforming preprocessor
		PRIVATE /Zc:referenceBinding # Enforce reference binding rules
		PRIVATE /volatile:iso # Enable standard-conforming interpretation of `volatile`
		PRIVATE /fp:fast # Enable aggressive floating-point optimizations
		PRIVATE $<${is_optimized_config}:/GS-> # Disable security checks
		PRIVATE $<${target_avx}:/arch:AVX> # Enable AVX instructions
		PRIVATE $<${target_avx2}:/arch:AVX2> # Enable AVX2 instructions
		PRIVATE $<${target_avx512}:/arch:AVX512> # Enable AVX-512 instructions
		PRIVATE $<${is_msvc_cl}:/MP> # Multi-threaded compilation
		PRIVATE $<${is_clang_cl}:/clang:${macro_prefix_option}> # Make `__FILE__` relative
		PRIVATE $<${is_clang_cl}:-Qunused-arguments> # Disable warnings about unused arguments
	)

	target_link_options(godot-jolt
		PRIVATE $<${is_distribution_config}:/PDBALTPATH:${pdb_file_name}> # Strip PDB path
	)
else()
	set(use_avx512 $<BOOL:${GDJ_USE_AVX512}>)
	set(use_avx2 $<BOOL:${GDJ_USE_AVX2}>)
	set(use_bmi1 $<BOOL:${GDJ_USE_BMI1}>)
	set(use_fma3 $<BOOL:${GDJ_USE_FMA3}>)
	set(use_f16c $<BOOL:${GDJ_USE_F16C}>)
	set(use_avx $<BOOL:${GDJ_USE_AVX}>)
	set(use_sse4_2 $<BOOL:${GDJ_USE_SSE4_2}>)
	set(use_sse2 $<BOOL:${GDJ_USE_SSE2}>)

	target_compile_options(godot-jolt
		PRIVATE -Wall # Enable common warnings
		PRIVATE -Wextra # Enable more common warnings
		PRIVATE -pedantic # Enable standard conformance warnings
		PRIVATE -Wconversion # Enable implicit conversion warnings
		PRIVATE -Wsign-conversion # Enable more implicit conversion warnings
		PRIVATE -Wcast-qual # Enable warnings about casting away qualifiers
		PRIVATE -Wshadow # Enable variable/type shadowing warnings
		PRIVATE -Wundef # Enable warnings about undefined identifiers in `#if` directives
		PRIVATE -pthread # Use POSIX threads
		PRIVATE -ffast-math # Enable aggressive floating-point optimizations
		PRIVATE ${macro_prefix_option} # Make `__FILE__` relative
		PRIVATE $<${use_sse2}:-msse2> # Enable SSE2 instructions
		PRIVATE $<${use_sse4_2}:-msse4.2> # Enable SSE4.2 instructions
		PRIVATE $<${use_sse4_2}:-mpopcnt> # Enable the POPCNT instruction
		PRIVATE $<${use_avx}:-mavx> # Enable AVX instructions
		PRIVATE $<${use_f16c}:-mf16c> # Enable F16C instructions
		PRIVATE $<${use_fma3}:-mfma> # Enable FMA3 instructions
		PRIVATE $<${use_bmi1}:-mbmi> # Enable BMI1 instructions
		PRIVATE $<${use_bmi1}:-mlzcnt> # Enable the LZCNT instruction
		PRIVATE $<${use_avx2}:-mavx2> # Enable AVX2 instructions
		PRIVATE $<${use_avx512}:-mavx512f> # Enable AVX-512 Foundation instructions
		PRIVATE $<${use_avx512}:-mavx512vl> # Enable AVX-512 Vector Length instructions
		PRIVATE $<${use_avx512}:-mavx512dq> # Enable AVX-512 Doubleword and Quadword instructions
		PRIVATE $<${is_gcc}:-no-integrated-cpp> # Workaround for GCC ignoring _Pragma (GCC#53431)
	)

	target_link_options(godot-jolt
		PRIVATE $<${use_static_crt}:-static-libgcc> # Link libgcc statically
		PRIVATE $<${use_static_crt}:-static-libstdc++> # Link libstdc++ statically
	)
endif()

install(
	TARGETS godot-jolt
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION bin
	FRAMEWORK DESTINATION bin
)

if(MSVC)
	# Non-distribution configurations will have the (default) PDB absolute paths embedded in them,
	# so we don't need to copy this file for a debugger to find them, since we're likely developing
	# locally anyway
	install(
		FILES ${pdb_file}
		CONFIGURATIONS Distribution EditorDistribution
		DESTINATION bin
		OPTIONAL
	)
endif()
