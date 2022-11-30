include_guard()

include(ExternalProject)

function(cmake_args_to_script variable args)
	set(arg_pattern [[^(.+)=(.*)$]])

	set(script_content "")
	set(script_line "")
	set(arg_rest "")

	foreach(element IN LISTS args)
		if(element MATCHES [[^-D(.*)]])
			set(arg ${CMAKE_MATCH_1})

			if(NOT script_line STREQUAL "")
				string(APPEND script_line "${arg_rest}\" CACHE INTERNAL \"\")")
				string(APPEND script_content "${script_line}\n")

				set(script_line "")
				set(arg_rest "")
			endif()

			if(arg MATCHES ${arg_pattern})
				set(arg_name ${CMAKE_MATCH_1})
				set(arg_value ${CMAKE_MATCH_2})
				set(script_line "set(${arg_name} \"${arg_value}")
			endif()
		else()
			string(APPEND arg_rest "\\\;${element}")
		endif()
	endforeach()

	if(NOT script_line STREQUAL "")
		string(APPEND script_line "${arg_rest}\" CACHE INTERNAL \"\")")
		string(APPEND script_content "${script_line}\n")
	endif()

	set(${variable} ${script_content} PARENT_SCOPE)
endfunction()

macro(escape_separator variable output_variable)
	string(REPLACE ";" $<SEMICOLON> ${output_variable} "${${variable}}")
endmacro()

function(GodotJoltExternalLibrary_Add library_name library_configs)
	# Declare our initial arguments

	set(one_value_args
		GIT_REPOSITORY
		GIT_COMMIT
		SOURCE_SUBDIR
		OUTPUT_NAME
	)

	set(multi_value_args
		INCLUDE_DIRECTORIES
		COMPILE_DEFINITIONS
		CMAKE_CACHE_ARGS
	)

	# Add a `LIBRARY_CONFIG_<CONFIG>` argument for each of the project's configurations

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		string(TOUPPER ${project_config} project_config_upper)
		list(APPEND one_value_args LIBRARY_CONFIG_${project_config_upper})
	endforeach()

	# Add a `OUTPUT_NAME_<CONFIG>` argument for each of the library's configurations

	foreach(library_config IN LISTS library_configs)
		string(TOUPPER ${library_config} library_config_upper)
		list(APPEND one_value_args OUTPUT_NAME_${library_config_upper})
	endforeach()

	# Add a `COMPILE_DEFINITIONS_<CONFIG>` argument for each of the library's configurations

	foreach(library_config IN LISTS library_configs)
		string(TOUPPER ${library_config} library_config_upper)
		list(APPEND multi_value_args COMPILE_DEFINITIONS_${library_config_upper})
	endforeach()

	# Parse arguments

	cmake_parse_arguments(arg
		""
		"${one_value_args}"
		"${multi_value_args}"
		${ARGN}
	)

	# Set default values for `OUTPUT_NAME_<CONFIG>` in case some weren't provided

	foreach(library_config IN LISTS library_configs)
		string(TOUPPER ${library_config} library_config_upper)

		if(NOT DEFINED arg_OUTPUT_NAME_${library_config_upper})
			if(DEFINED arg_OUTPUT_NAME)
				set(arg_OUTPUT_NAME_${library_config_upper} ${arg_OUTPUT_NAME})
			else()
				message(FATAL_ERROR "Expected configuration-agnostic OUTPUT_NAME")
			endif()
		endif()
	endforeach()

	# Set up the directories

	get_directory_property(ep_base EP_BASE)

	set(binary_dir ${ep_base}/Build/${library_name})
	set(download_dir ${ep_base}/Download/${library_name})
	set(install_dir ${ep_base}/Install/${library_name})
	set(source_dir ${ep_base}/Source/${library_name})
	set(stamp_dir ${ep_base}/Stamp/${library_name})
	set(tmp_dir ${ep_base}/tmp/${library_name})

	# Resolve any placeholder paths

	foreach(arg arg_INCLUDE_DIRECTORIES)
		string(REPLACE <BINARY_DIR> ${binary_dir} ${arg} "${${arg}}")
		string(REPLACE <DOWNLOAD_DIR> ${download_dir} ${arg} "${${arg}}")
		string(REPLACE <INSTALL_DIR> ${install_dir} ${arg} "${${arg}}")
		string(REPLACE <SOURCE_DIR> ${source_dir} ${arg} "${${arg}}")
		string(REPLACE <STAMP_DIR> ${stamp_dir} ${arg} "${${arg}}")
		string(REPLACE <TMP_DIR> ${tmp_dir} ${arg} "${${arg}}")
	endforeach()

	# Set up mappings between configurations. For library config `Foo` that maps to project config
	# `Bar` we get `set(project_config_Foo Bar)` and `set(library_config_Bar Foo)`.

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		string(TOUPPER ${project_config} project_config_upper)
		set(library_config ${arg_LIBRARY_CONFIG_${project_config_upper}})
		set(library_config_${project_config} ${library_config})
		set(project_config_${library_config} ${project_config})
	endforeach()

	# Set up an agnostic library config using generator expressions that can be used for stuff like
	# `BUILD_BYPRODUCTS`

	set(library_config_current "")

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		set(condition $<CONFIG:${project_config}>)
		set(library_config ${library_config_${project_config}})
		string(APPEND library_config_current $<${condition}:${library_config}>)
	endforeach()

	# Set up the paths to the output directories. For library config `Foo` that maps to project
	# config `Bar` we get `set(output_dir_Bar ${binary_dir}/Foo)`.

	get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		if(is_multi_config)
			set(library_config ${library_config_${project_config}})
			set(output_dir_${project_config} ${binary_dir}/${library_config})
		else()
			set(output_dir_${project_config} ${binary_dir})
		endif()
	endforeach()

	# Set up an agnostic output directory using generator expressions that can be used for stuff
	# like `BUILD_BYPRODUCTS`

	if(is_multi_config)
		set(output_dir_current ${binary_dir}/${library_config_current})
	else()
		set(output_dir_current ${binary_dir})
	endif()

	# Set up the output names. For library config `Foo` that map to project config `Bar` we get
	# `set(output_name_Bar ${arg_OUTPUT_NAME_FOO})`.

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		set(library_config ${library_config_${project_config}})
		string(TOUPPER ${library_config} library_config_upper)
		string(CONCAT output_name_${project_config}
			${CMAKE_STATIC_LIBRARY_PREFIX}
			${arg_OUTPUT_NAME_${library_config_upper}}
			${CMAKE_STATIC_LIBRARY_SUFFIX}
		)
	endforeach()

	# Set up an agnostic output name using generator expressions that can be used for stuff like
	# `BUILD_BYPRODUCTS`

	set(output_name_current "")

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		set(condition $<CONFIG:${project_config}>)
		set(output_name ${output_name_${project_config}})
		string(APPEND output_name_current $<${condition}:${output_name}>)
	endforeach()

	# Set up the CMake arguments, most of which go through an "initial cache file" in order to defer
	# evaluation of generator expressions to the external project

	set(use_static_crt $<BOOL:${GDJOLT_STATIC_RUNTIME_LIBRARY}>)
	set(msvcrt_debug $<$<CONFIG:${library_config_Debug},${library_config_EditorDebug}>:Debug>)
	set(msvcrt_dll $<$<NOT:${use_static_crt}>:DLL>)
	set(msvcrt MultiThreaded${msvcrt_debug}${msvcrt_dll})

	set(cmake_cache_args
		-DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
		-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
		-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
		-DCMAKE_POSITION_INDEPENDENT_CODE=TRUE
		-DCMAKE_POLICY_DEFAULT_CMP0069=NEW # Allows use of INTERPROCEDURAL_OPTIMIZATION
		-DCMAKE_POLICY_DEFAULT_CMP0091=NEW # Allows use of MSVC_RUNTIME_LIBRARY
		-DCMAKE_MSVC_RUNTIME_LIBRARY=${msvcrt}
		${arg_CMAKE_CACHE_ARGS}
	)

	if(CMAKE_HOST_SYSTEM_NAME STREQUAL Darwin)
		list(APPEND cmake_cache_args
			-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}
			-DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET}
		)
	endif()

	set(cache_file ${tmp_dir}/${library_name}-cache.cmake)
	cmake_args_to_script(cache_file_content "${cmake_cache_args}")
	file(CONFIGURE OUTPUT ${cache_file} CONTENT ${cache_file_content})

	# We pass CMAKE_BUILD_TYPE through the command-line like normal since we need its generator
	# expressions to evaluate here and not in the external project

	if(is_multi_config)
		set(build_type_arg "")
	else()
		set(build_type_arg -DCMAKE_BUILD_TYPE=${library_config_current})
	endif()

	# Finally declare the ExternalProject, which will be the one doing the actual work

	ExternalProject_Add(${library_name}
		GIT_REPOSITORY ${arg_GIT_REPOSITORY}
		GIT_TAG ${arg_GIT_COMMIT}
		SOURCE_SUBDIR ${arg_SOURCE_SUBDIR}
		CMAKE_ARGS -C ${cache_file} ${build_type_arg}
		BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${library_config_current}
		INSTALL_COMMAND ""
		BUILD_BYPRODUCTS ${output_dir_current}/${output_name_current}
		EXCLUDE_FROM_ALL TRUE # These will always be dependencies anyway
		UPDATE_DISCONNECTED TRUE # See extra steps below
		STEP_TARGETS update # See extra steps below
	)

	# Ensure that the external project runs its `configure` step if the cache file created above
	# ever changes, by adding an empty custom step that depends on that file and sits inbetween
	# `patch` and `configure`

	ExternalProject_Add_Step(${library_name} configure-if-different
		COMMAND ""
		DEPENDS ${cache_file}
		DEPENDEES patch
		DEPENDERS configure
	)

	# Ensure that the external project runs its (now disconnected) `update` step if the Git commit
	# ever changes, by writing it to a file at configure-time and adding that file as a dependency
	# to a custom step that explicitly invokes the `update` step target. This custom step sits
	# inbetween `download` and `configure`, just like `update` did before it was disconnected.

	set(commit_file ${stamp_dir}/${library_name}-gitcommit.txt)
	set(commit_file_content ${arg_GIT_COMMIT})
	file(CONFIGURE OUTPUT ${commit_file} CONTENT ${commit_file_content})

	ExternalProject_Add_Step(${library_name} update-if-different
		COMMAND ${CMAKE_COMMAND}
			--build ${CMAKE_CURRENT_BINARY_DIR}
			--config $<CONFIG>
			--target ${library_name}-update
		DEPENDS ${commit_file}
		DEPENDEES download
		DEPENDERS configure
	)

	# Declare the shim library that we will consume in the end

	# HACK(mihe): Include directories need to exist at configure time
	file(MAKE_DIRECTORY ${arg_INCLUDE_DIRECTORIES})

	set(target_name ${PROJECT_NAME}_${library_name})
	set(target_alias ${PROJECT_NAME}::${library_name})

	add_library(${target_name} STATIC IMPORTED)
	add_library(${target_alias} ALIAS ${target_name})
	add_dependencies(${target_name} ${library_name})

	# Set up the target properties

	escape_separator(library_configs library_configs_)
	escape_separator(arg_INCLUDE_DIRECTORIES include_directories_)

	set(target_properties
		IMPORTED_LINK_INTERFACE_LANGUAGES CXX
		IMPORTED_CONFIGURATIONS ${library_configs_}
		INTERFACE_INCLUDE_DIRECTORIES ${include_directories_}
	)

	# Add the compile definitions to the target properties, both generic and config-specific ones

	set(compile_definitions ${arg_COMPILE_DEFINITIONS})

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		set(library_config ${library_config_${project_config}})
		string(TOUPPER ${library_config} library_config_upper)
		set(condition $<CONFIG:${project_config}>)

		foreach(define IN LISTS arg_COMPILE_DEFINITIONS_${library_config_upper})
			list(APPEND compile_definitions $<${condition}:${define}>)
		endforeach()
	endforeach()

	if(NOT "${compile_definitions}" STREQUAL "")
		escape_separator(compile_definitions compile_definitions_)
		list(APPEND target_properties INTERFACE_COMPILE_DEFINITIONS "${compile_definitions_}")
	endif()

	# Add the target property that maps library configurations to project configurations. For
	# project config `Foo` that maps to library config `Bar` we get `MAP_IMPORTED_CONFIG_FOO Bar`.

	foreach(project_config IN LISTS GDJOLT_CONFIGURATIONS)
		string(TOUPPER ${project_config} project_config_upper)
		set(library_config ${library_config_${project_config}})
		list(APPEND target_properties MAP_IMPORTED_CONFIG_${project_config_upper} ${library_config})
	endforeach()

	# Add the target property that declares the output path. For library config `Foo` that maps to
	# project config `Bar` we get `IMPORTED_LOCATION_FOO ${output_dir_Bar}/${output_name_Bar}`.

	foreach(library_config IN LISTS library_configs)
		string(TOUPPER ${library_config} library_config_upper)
		set(project_config ${project_config_${library_config}})
		set(output_dir ${output_dir_${project_config}})
		set(output_name ${output_name_${project_config}})
		set(output_path ${output_dir}/${output_name})
		list(APPEND target_properties IMPORTED_LOCATION_${library_config_upper} ${output_path})
	endforeach()

	set_target_properties(${target_name} PROPERTIES ${target_properties})
endfunction()
