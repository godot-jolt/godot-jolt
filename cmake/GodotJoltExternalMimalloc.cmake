include_guard()

include(GodotJoltExternalLibrary)

set(configurations
	Debug
	RelWithDebInfo
)

if(WIN32)
	set(output_name mimalloc-static)
else()
	set(output_name mimalloc)
endif()

if(MSVC)
	set(nowarn_option /W0)
else()
	set(nowarn_option -w)
endif()

GodotJoltExternalLibrary_Add(mimalloc "${configurations}"
	GIT_REPOSITORY https://github.com/godot-jolt/mimalloc.git
	GIT_COMMIT 91ba1f374da66e624841f53f6659da3a8f8f93ea
	OUTPUT_NAME ${output_name}
	INCLUDE_DIRECTORIES
		<SOURCE_DIR>/include
	CMAKE_CACHE_ARGS
		-DCMAKE_C_FLAGS=${nowarn_option}
		-DCMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO=${GDJOLT_LTO}
		-DMI_OVERRIDE=FALSE
		-DMI_USE_CXX=FALSE
		-DMI_OSX_INTERPOSE=FALSE
		-DMI_OSX_ZONE=FALSE
		-DMI_WIN_REDIRECT=FALSE
		-DMI_BUILD_SHARED=FALSE
		-DMI_BUILD_OBJECT=FALSE
		-DMI_BUILD_TESTS=FALSE
		-DMI_SKIP_COLLECT_ON_EXIT=TRUE
	LIBRARY_CONFIG_DEBUG Debug
	LIBRARY_CONFIG_DEVELOPMENT RelWithDebInfo
	LIBRARY_CONFIG_DISTRIBUTION RelWithDebInfo
	LIBRARY_CONFIG_EDITORDEBUG Debug
	LIBRARY_CONFIG_EDITORDEVELOPMENT RelWithDebInfo
	LIBRARY_CONFIG_EDITORDISTRIBUTION RelWithDebInfo
)
