include_guard()

include(GodotJoltUtilities)

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

gdjolt_duplicate_config(RelWithDebInfo Development)
gdjolt_duplicate_config(RelWithDebInfo Distribution)
gdjolt_duplicate_config(Debug EditorDebug)
gdjolt_duplicate_config(Development EditorDevelopment)
gdjolt_duplicate_config(Distribution EditorDistribution)

if(PROJECT_IS_TOP_LEVEL)
	gdjolt_remove_config(MinSizeRel)
	gdjolt_remove_config(Release)
	gdjolt_remove_config(RelWithDebInfo)
endif()
