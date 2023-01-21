#include "jolt_hooks.hpp"

void* jolt_alloc(size_t p_size) {
	return mi_malloc(p_size);
}

void jolt_free(void* p_mem) {
	mi_free(p_mem);
}

void* jolt_aligned_alloc(size_t p_size, size_t p_alignment) {
	return mi_aligned_alloc(p_alignment, p_size);
}

void jolt_aligned_free(void* p_mem) {
	mi_free(p_mem);
}

void jolt_trace(const char* p_format, ...) {
	va_list args; // NOLINT(cppcoreguidelines-init-variables)
	va_start(args, p_format);
	char buffer[1024] = {'\0'};
	vsnprintf(buffer, sizeof(buffer), p_format, args);
	va_end(args);
	UtilityFunctions::print_verbose(buffer);
}

#ifdef JPH_ENABLE_ASSERTS

bool jolt_assert(const char* p_expr, const char* p_msg, const char* p_file, uint32_t p_line) {
	ERR_PRINT(vformat(
		"Assertion '%s' failed with message '%s' at '%s:%d'",
		p_expr,
		p_msg != nullptr ? p_msg : "",
		p_file,
		p_line
	));

	CRASH_NOW();

	return false;
}

#endif // JPH_ENABLE_ASSERTS

void initialize_jolt_hooks() {
	JPH::Allocate = &jolt_alloc;
	JPH::Free = &jolt_free;
	JPH::AlignedAllocate = &jolt_aligned_alloc;
	JPH::AlignedFree = &jolt_aligned_free;

	JPH::Trace = &jolt_trace;

	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = jolt_assert;)

	JPH::Factory::sInstance = new JPH::Factory();

	JPH::RegisterTypes();
}

void deinitialize_jolt_hooks() {
	delete_safely(JPH::Factory::sInstance);
}
