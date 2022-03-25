#ifndef DESCARTES_MACROS_H
#define DESCARTES_MACROS_H
// clang-format off

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED(X) __attribute__((deprecated(X)))
#elif defined(_MSC_VER)
#define DEPRECATED(X) __declspec(deprecated(X))
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(X)
#endif

#if defined(__clang__)
#define DESCARTES_IGNORE_WARNINGS_PUSH				\
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wall\"") \
  _Pragma("GCC diagnostic ignored \"-Wint-to-pointer-cast\"")		\
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")		\
  _Pragma("GCC diagnostic ignored \"-Winconsistent-missing-override\"")	\
  _Pragma("GCC diagnostic ignored \"-Wconversion\"")			\
  _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"")		\
  _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")		\
  _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")

#define DESCARTES_IGNORE_WARNINGS_POP _Pragma("GCC diagnostic pop")
#elif defined(__GNUC__)
#define DESCARTES_IGNORE_WARNINGS_PUSH				\
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wall\"") \
  _Pragma("GCC diagnostic ignored \"-Wint-to-pointer-cast\"")		\
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")		\
  _Pragma("GCC diagnostic ignored \"-Wsuggest-override\"")		\
  _Pragma("GCC diagnostic ignored \"-Wconversion\"")			\
  _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"")		\
  _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")		\
  _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")

#define DESCARTES_IGNORE_WARNINGS_POP _Pragma("GCC diagnostic pop")
#elif defined(_MSC_VER)
#define DESCARTES_IGNORE_WARNINGS_PUSH
#define DESCARTES_IGNORE_WARNINGS_POP
#else
#pragma message("WARNING: You need to implement DESCARTES_IGNORE_WARNINGS_PUSH and DESCARTES_IGNORE_WARNINGS_POP for this compiler")
#endif

// clang-format on
#endif  // DESCARTES_MACROS_H
