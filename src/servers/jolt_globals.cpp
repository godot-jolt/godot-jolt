#include "jolt_globals.hpp"

#include "objects/jolt_group_filter.hpp"
#include "shapes/jolt_custom_double_sided_shape.hpp"
#include "shapes/jolt_custom_empty_shape.hpp"
#include "shapes/jolt_custom_ray_shape.hpp"
#include "shapes/jolt_custom_user_data_shape.hpp"

#ifdef GDJ_CONFIG_EDITOR
#include "servers/jolt_physics_server_3d.hpp"
#include <unordered_map>

std::unordered_map<void*, size_t> alloc_size_map;
#endif // GDJ_CONFIG_EDITOR

#ifdef GDJ_USE_MIMALLOC
#include <mimalloc-new-delete.h>
#endif // GDJ_USE_MIMALLOC

void* jolt_alloc(size_t p_size) {
#ifdef GDJ_USE_MIMALLOC
	void* block = mi_malloc(p_size);
#else
	void* block = malloc(p_size);
#endif // GDJ_USE_MIMALLOC

#ifdef GDJ_CONFIG_EDITOR
	if (block != nullptr){
		JoltPhysicsServer3D::update_current_memory((int64_t)p_size);
		alloc_size_map.insert({block, p_size});
	}
#endif // GDJ_CONFIG_EDITOR

	return block;
}

void jolt_free(void* p_mem) {
#ifdef GDJ_USE_MIMALLOC
	mi_free(p_mem);
#else
	free(p_mem);
#endif // GDJ_USE_MIMALLOC

#ifdef GDJ_CONFIG_EDITOR
	JoltPhysicsServer3D::update_current_memory(-(int64_t)alloc_size_map.at(p_mem));
	alloc_size_map.erase(p_mem);
#endif // GDJ_CONFIG_EDITOR
}

void* jolt_aligned_alloc(size_t p_size, size_t p_alignment) {
#ifdef GDJ_USE_MIMALLOC
	void* block = mi_malloc_aligned(p_size, p_alignment);
#else

#ifdef JPH_PLATFORM_WINDOWS
	void* block = _aligned_malloc(p_size, p_alignment);
#else
	void* block = nullptr;
	posix_memalign(&block, p_alignment, p_size);
#endif // JPH_PLATFORM_WINDOWS

#endif // GDJ_USE_MIMALLOC

#ifdef GDJ_CONFIG_EDITOR
	if (block != nullptr) {
		// I don't know how to calculate effective allocation size of aligned alloc
		JoltPhysicsServer3D::update_current_memory((int64_t)p_size);
		alloc_size_map.insert({block, p_size});
	}
#endif // GDJ_CONFIG_EDITOR

	return block;
}

void jolt_aligned_free(void* p_mem) {
#ifdef GDJ_USE_MIMALLOC
	mi_free(p_mem);
#else

#ifdef JPH_PLATFORM_WINDOWS
	_aligned_free(p_mem);
#else
	free(p_mem);
#endif // JPH_PLATFORM_WINDOWS

#endif // GDJ_USE_MIMALLOC

#ifdef GDJ_CONFIG_EDITOR
	JoltPhysicsServer3D::update_current_memory(-(int64_t)alloc_size_map.at(p_mem));
	alloc_size_map.erase(p_mem);
#endif // GDJ_CONFIG_EDITOR
}

#ifdef JPH_ENABLE_ASSERTS

void jolt_trace(const char* p_format, ...) {
	// NOLINTNEXTLINE(cppcoreguidelines-init-variables)
	va_list args;
	va_start(args, p_format);
	char buffer[1024] = {'\0'};
	vsnprintf(buffer, sizeof(buffer), p_format, args);
	va_end(args);
	UtilityFunctions::print_verbose(buffer);
}

bool jolt_assert(const char* p_expr, const char* p_msg, const char* p_file, uint32_t p_line) {
	CRASH_NOW_MSG(vformat(
		"Assertion '%s' failed with message '%s' at '%s:%d'",
		p_expr,
		p_msg != nullptr ? p_msg : "",
		p_file,
		p_line
	));

	return false;
}

#endif // JPH_ENABLE_ASSERTS

void jolt_initialize() {
	JPH::Allocate = &jolt_alloc;
	JPH::Free = &jolt_free;
	JPH::AlignedAllocate = &jolt_aligned_alloc;
	JPH::AlignedFree = &jolt_aligned_free;

#ifdef JPH_ENABLE_ASSERTS
	JPH::Trace = &jolt_trace;
	JPH::AssertFailed = jolt_assert;
#endif // JPH_ENABLE_ASSERTS

	JPH::Factory::sInstance = new JPH::Factory();

	JPH::RegisterTypes();

	JoltCustomEmptyShape::register_type();
	JoltCustomRayShape::register_type();
	JoltCustomUserDataShape::register_type();
	JoltCustomDoubleSidedShape::register_type();

	JoltGroupFilter::instance = new JoltGroupFilter();
	JoltGroupFilter::instance->SetEmbedded();
}

void jolt_deinitialize() {
	delete_safely(JoltGroupFilter::instance);

	JPH::UnregisterTypes();

	delete_safely(JPH::Factory::sInstance);
}
