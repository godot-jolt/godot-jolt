#pragma once

#ifdef _MSC_VER
#define GDCLASS_PRAGMA(x) __pragma(x)

// C4100: unreferenced formal parameter
#define GDCLASS_DISABLE_WARNINGS  \
	GDCLASS_PRAGMA(warning(push)) \
	GDCLASS_PRAGMA(warning(disable : 4100))

#define GDCLASS_RESTORE_WARNINGS GDCLASS_PRAGMA(warning(pop))
#endif // _MSC_VER

#ifdef __GNUC__
#define GDCLASS_PRAGMA(x) _Pragma(#x)

#define GDCLASS_DISABLE_WARNINGS        \
	GDCLASS_PRAGMA(GCC diagnostic push) \
	GDCLASS_PRAGMA(GCC diagnostic ignored "-Wunused-parameter")

#define GDCLASS_RESTORE_WARNINGS GDCLASS_PRAGMA(GCC diagnostic pop)
#endif // __GNUC__

// TODO(mihe): When LLVM 15 becomes the norm this macro can be NOLINT'ed instead of having to do it
// at every single call site.
#define GDCLASS_NO_WARN(m_class, m_inherits) \
	GDCLASS_DISABLE_WARNINGS                 \
	GDCLASS(m_class, m_inherits)             \
	GDCLASS_RESTORE_WARNINGS
