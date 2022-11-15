include_guard()

set(GDJOLT_CONFIGURATIONS
	Debug
	Development
	Distribution
	EditorDebug
	EditorDevelopment
	EditorDistribution
)

get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

if(is_multi_config)
	if(DEFINED CMAKE_BUILD_TYPE)
		message(FATAL_ERROR "CMAKE_BUILD_TYPE is not compatible with multi-config generators.")
	endif()

	if(PROJECT_IS_TOP_LEVEL)
		set(CMAKE_CONFIGURATION_TYPES ${GDJOLT_CONFIGURATIONS} CACHE STRING
			"Semicolon separated list of supported configuration types."
			FORCE
		)
	endif()

	foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
		if(NOT config IN_LIST GDJOLT_CONFIGURATIONS)
			message(FATAL_ERROR "Unsupported configuration: '${config}'.")
		endif()
	endforeach()
else()
	if(DEFINED CMAKE_CONFIGURATION_TYPES)
		message(FATAL_ERROR "CMAKE_CONFIGURATION_TYPES is not compatible with single-config generators.")
	endif()

	if(NOT CMAKE_BUILD_TYPE)
		message(FATAL_ERROR "No build type specified.")
	endif()

	if(NOT CMAKE_BUILD_TYPE IN_LIST GDJOLT_CONFIGURATIONS)
		message(FATAL_ERROR "Unsupported build type: '${CMAKE_BUILD_TYPE}'.")
	endif()

	if(PROJECT_IS_TOP_LEVEL)
		set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${GDJOLT_CONFIGURATIONS})
	endif()
endif()

macro(duplicate_config src dst)
	string(TOUPPER ${src} src_upper)
	string(TOUPPER ${dst} dst_upper)

	set(CMAKE_CXX_FLAGS_${dst_upper} ${CMAKE_CXX_FLAGS_${src_upper}} CACHE STRING
		"Flags used by the CXX compiler during ${dst_upper} builds."
	)

	set(CMAKE_EXE_LINKER_FLAGS_${dst_upper} ${CMAKE_EXE_LINKER_FLAGS_${src_upper}} CACHE STRING
		"Flags used by the linker during ${dst_upper} builds."
	)

	set(CMAKE_MODULE_LINKER_FLAGS_${dst_upper} ${CMAKE_MODULE_LINKER_FLAGS_${src_upper}} CACHE STRING
		"Flags used by the linker during the creation of modules during ${dst_upper} builds."
	)

	set(CMAKE_SHARED_LINKER_FLAGS_${dst_upper} ${CMAKE_SHARED_LINKER_FLAGS_${src_upper}} CACHE STRING
		"Flags used by the linker during the creation of shared libraries during ${dst_upper} builds."
	)

	set(CMAKE_STATIC_LINKER_FLAGS_${dst_upper} ${CMAKE_STATIC_LINKER_FLAGS_${src_upper}} CACHE STRING
		"Flags used by the linker during the creation of static libraries during ${dst_upper} builds."
	)

	mark_as_advanced(
		CMAKE_CXX_FLAGS_${dst_upper}
		CMAKE_EXE_LINKER_FLAGS_${dst_upper}
		CMAKE_MODULE_LINKER_FLAGS_${dst_upper}
		CMAKE_SHARED_LINKER_FLAGS_${dst_upper}
		CMAKE_STATIC_LINKER_FLAGS_${dst_upper}
	)

	if(MSVC)
		set(CMAKE_RC_FLAGS_${dst_upper} ${CMAKE_RC_FLAGS_${src_upper}} CACHE STRING
			"Flags for Windows Resource Compiler during ${dst_upper} builds."
		)

		mark_as_advanced(CMAKE_RC_FLAGS_${dst_upper})
	endif()
endmacro()

macro(remove_config name)
	string(TOUPPER ${name} name_upper)

	unset(CMAKE_CXX_FLAGS_${name_upper} CACHE)
	unset(CMAKE_EXE_LINKER_FLAGS_${name_upper} CACHE)
	unset(CMAKE_MODULE_LINKER_FLAGS_${name_upper} CACHE)
	unset(CMAKE_SHARED_LINKER_FLAGS_${name_upper} CACHE)
	unset(CMAKE_STATIC_LINKER_FLAGS_${name_upper} CACHE)

	if(MSVC)
		unset(CMAKE_RC_FLAGS_${name_upper} CACHE)
	endif()
endmacro()

duplicate_config(RelWithDebInfo Development)
duplicate_config(RelWithDebInfo Distribution)
duplicate_config(Debug EditorDebug)
duplicate_config(Development EditorDevelopment)
duplicate_config(Distribution EditorDistribution)

if(PROJECT_IS_TOP_LEVEL)
	remove_config(MinSizeRel)
	remove_config(Release)
	remove_config(RelWithDebInfo)
endif()
