include_guard()

macro(gdjolt_escape_separator variable output_variable)
	string(REPLACE ";" $<SEMICOLON> ${output_variable} "${${variable}}")
endmacro()

macro(gdjolt_duplicate_config src dst)
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

macro(gdjolt_remove_config name)
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

function(gdjolt_args_to_script variable args)
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
