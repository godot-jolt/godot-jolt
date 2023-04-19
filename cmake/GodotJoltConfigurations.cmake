include_guard()

include(GodotJoltUtilities)

set(GJ_CONFIGURATIONS
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
		set(CMAKE_CONFIGURATION_TYPES ${GJ_CONFIGURATIONS} CACHE STRING
			"Semicolon separated list of supported configuration types."
			FORCE
		)
	endif()

	foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
		if(NOT config IN_LIST GJ_CONFIGURATIONS)
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

	if(NOT CMAKE_BUILD_TYPE IN_LIST GJ_CONFIGURATIONS)
		message(FATAL_ERROR "Unsupported build type: '${CMAKE_BUILD_TYPE}'.")
	endif()

	if(PROJECT_IS_TOP_LEVEL)
		set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${GJ_CONFIGURATIONS})
	endif()
endif()

gj_duplicate_config(RelWithDebInfo Development)
gj_duplicate_config(RelWithDebInfo Distribution)
gj_duplicate_config(Debug EditorDebug)
gj_duplicate_config(Development EditorDevelopment)
gj_duplicate_config(Distribution EditorDistribution)

if(PROJECT_IS_TOP_LEVEL)
	gj_remove_config(MinSizeRel)
	gj_remove_config(Release)
	gj_remove_config(RelWithDebInfo)
endif()
