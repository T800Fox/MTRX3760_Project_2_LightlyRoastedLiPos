#ifndef MTRX3760_LRL_WAREHOUSEBOT__VISIBILITY_CONTROL_H_
#define MTRX3760_LRL_WAREHOUSEBOT__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MTRX3760_LRL_WAREHOUSEBOT_EXPORT __attribute__ ((dllexport))
    #define MTRX3760_LRL_WAREHOUSEBOT_IMPORT __attribute__ ((dllimport))
  #else
    #define MTRX3760_LRL_WAREHOUSEBOT_EXPORT __declspec(dllexport)
    #define MTRX3760_LRL_WAREHOUSEBOT_IMPORT __declspec(dllimport)
  #endif
  #ifdef MTRX3760_LRL_WAREHOUSEBOT_BUILDING_DLL
    #define MTRX3760_LRL_WAREHOUSEBOT_PUBLIC MTRX3760_LRL_WAREHOUSEBOT_EXPORT
  #else
    #define MTRX3760_LRL_WAREHOUSEBOT_PUBLIC MTRX3760_LRL_WAREHOUSEBOT_IMPORT
  #endif
  #define MTRX3760_LRL_WAREHOUSEBOT_PUBLIC_TYPE MTRX3760_LRL_WAREHOUSEBOT_PUBLIC
  #define MTRX3760_LRL_WAREHOUSEBOT_LOCAL
#else
  #define MTRX3760_LRL_WAREHOUSEBOT_EXPORT __attribute__ ((visibility("default")))
  #define MTRX3760_LRL_WAREHOUSEBOT_IMPORT
  #if __GNUC__ >= 4
    #define MTRX3760_LRL_WAREHOUSEBOT_PUBLIC __attribute__ ((visibility("default")))
    #define MTRX3760_LRL_WAREHOUSEBOT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MTRX3760_LRL_WAREHOUSEBOT_PUBLIC
    #define MTRX3760_LRL_WAREHOUSEBOT_LOCAL
  #endif
  #define MTRX3760_LRL_WAREHOUSEBOT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MTRX3760_LRL_WAREHOUSEBOT__VISIBILITY_CONTROL_H_