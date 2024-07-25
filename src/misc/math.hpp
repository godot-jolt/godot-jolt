#pragma once

// NOLINTBEGIN(readability-identifier-naming)

#define Mathf_SQRT12 ((float)Math_SQRT12)
#define Mathf_SQRT2 ((float)Math_SQRT2)
#define Mathf_LN2 ((float)Math_LN2)
#define Mathf_PI ((float)Math_PI)
#define Mathf_TAU ((float)Math_TAU)
#define Mathf_E ((float)Math_E)
#define Mathf_INF ((float)Math_INF)
#define Mathf_NAN ((float)Math_NAN)

// NOLINTEND(readability-identifier-naming)

#define USEC_TO_SEC(m_usec) (double(m_usec) / 1000000.0)

#ifdef GDJ_CONFIG_EDITOR

#define ENSURE_SCALE_NOT_ZERO(m_transform, m_msg)                                             \
	if (unlikely((m_transform).basis.determinant() == 0.0f)) {                                \
		WARN_PRINT(vformat(                                                                   \
			"%s "                                                                             \
			"The basis of the transform was singular, which is not supported by Godot Jolt. " \
			"This is likely caused by one or more axes having a scale of zero. "              \
			"The basis (and thus its scale) will be treated as identity.",                    \
			m_msg                                                                             \
		));                                                                                   \
                                                                                              \
		(m_transform).basis = Basis();                                                        \
	} else                                                                                    \
		((void)0)

#else // GDJ_CONFIG_EDITOR

#define ENSURE_SCALE_NOT_ZERO(m_transform, m_msg)

#endif // GDJ_CONFIG_EDITOR

namespace godot::Math {

void decompose(Basis& p_basis, Vector3& p_scale);

_FORCE_INLINE_ void decompose(Transform3D& p_transform, Vector3& p_scale) {
	decompose(p_transform.basis, p_scale);
}

_FORCE_INLINE_ Basis decomposed(Basis p_basis, Vector3& p_scale) {
	decompose(p_basis, p_scale);
	return p_basis;
}

_FORCE_INLINE_ Transform3D decomposed(Transform3D p_transform, Vector3& p_scale) {
	decompose(p_transform, p_scale);
	return p_transform;
}

_FORCE_INLINE_ float square(float p_value) {
	return p_value * p_value;
}

} // namespace godot::Math
