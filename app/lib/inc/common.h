#ifndef SCH_COMMON_H
#define SCH_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*--Weak macro definition----------------------------------------------------*/

/**
 * @brief 
 * 
 */
#if defined(__GNUC__)
  #define __WEAK __attribute__((weak))
#elif defined(__ICCARM__)
  #define __WEAK __weak
#elif defined(__ARMCC_VERSION)
  #define __WEAK __weak
#else
  #define __WEAK
#endif

#ifdef __cplusplus
}
#endif

#endif