#pragma once

#define ERR_FAIL_INDEX_D(m_index, m_size) ERR_FAIL_INDEX_V(m_index, m_size, {})
#define ERR_FAIL_INDEX_D_MSG(m_index, m_size, m_msg) \
	ERR_FAIL_INDEX_V_MSG(m_index, m_size, {}, m_msg)
#define ERR_FAIL_UNSIGNED_INDEX_D(m_index, m_size) ERR_FAIL_UNSIGNED_INDEX_V(m_index, m_size, {})
#define ERR_FAIL_UNSIGNED_INDEX_D_MSG(m_index, m_size, m_msg) \
	ERR_FAIL_UNSIGNED_INDEX_V_MSG(m_index, m_size, {}, m_msg)
#define ERR_FAIL_NULL_D(m_param) ERR_FAIL_NULL_V(m_param, {})
#define ERR_FAIL_NULL_D_MSG(m_param, m_msg) ERR_FAIL_NULL_V_MSG(m_param, {}, m_msg)
#define ERR_FAIL_COND_D(m_cond) ERR_FAIL_COND_V(m_cond, {})
#define ERR_FAIL_COND_D_MSG(m_cond, m_msg) ERR_FAIL_COND_V_MSG(m_cond, {}, m_msg)
#define ERR_FAIL_D() ERR_FAIL_V({})
#define ERR_FAIL_D_MSG(m_msg) ERR_FAIL_V_MSG({}, m_msg)

#define GDJOLT_MSG_NOT_IMPL vformat("%s is not implemented in Godot Jolt.", __FUNCTION__)
#define ERR_FAIL_NOT_IMPL() ERR_FAIL_MSG(GDJOLT_MSG_NOT_IMPL)
#define ERR_FAIL_V_NOT_IMPL(m_retval) ERR_FAIL_V_MSG(m_retval, GDJOLT_MSG_NOT_IMPL)
#define ERR_FAIL_D_NOT_IMPL() ERR_FAIL_D_MSG(GDJOLT_MSG_NOT_IMPL)
#define ERR_BREAK_NOT_IMPL() ERR_BREAK_MSG(GDJOLT_MSG_NOT_IMPL)
#define ERR_CONTINUE_NOT_IMPL() ERR_CONTINUE_MSG(GDJOLT_MSG_NOT_IMPL)
