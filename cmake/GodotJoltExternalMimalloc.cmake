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
	set(c_flags /W0)
else()
	set(c_flags -w)
endif()

if(DEFINED ENV{CFLAGS})
	set(c_flags "$ENV{CFLAGS} ${c_flags}")
endif()

GodotJoltExternalLibrary_Add(mimalloc "${configurations}"
	GIT_REPOSITORY https://github.com/godot-jolt/mimalloc.git
	GIT_COMMIT 91ba1f374da66e624841f53f6659da3a8f8f93ea
	LANGUAGE C
	OUTPUT_NAME ${output_name}
	INCLUDE_DIRECTORIES
		<SOURCE_DIR>/include
	CMAKE_CACHE_ARGS
		-DCMAKE_C_FLAGS=${c_flags}
		-DCMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO=${GDJOLT_INTERPROCEDURAL_OPTIMIZATION}
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
