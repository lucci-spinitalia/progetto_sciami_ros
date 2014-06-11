/**
 *\brief Useful auxiliary library.
 *This library provides a bunch of auxiliary c functions useful in everyday programming
 */
#ifndef _FUNZ_AUSILIARIE_
#define _FUNZ_AUSILIARIE_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*Macro MAX function*/
#define		MAX(a,b)((a>b)?a:b)

/**
 * \brief Minimum integer function
 * \param a first integer
 * \param b second integer
 * \return minimum integer
 */
inline int min_int(int a, int b);
/**
 * \brief Maximum integer function
 * \param a first integer
 * \param b second integer
 * \return maximum integer
 */
inline int max_int(int a, int b);
/**
 * \brief Minimum unsigned integer function
 * \param a first integer
 * \param b second integer
 * \return minimum integer
 */
inline unsigned int min_uint(unsigned int a, unsigned int b);
/**
 * \brief Maximum unsigned integer function
 * \param a first integer
 * \param b second integer
 * \return maximum integer
 */
inline unsigned int max_uint(unsigned int a, unsigned int b);
/**
 * \brief Minimum float function
 * \param a first float
 * \param b second float
 * \return minimum float
 */
inline float min(float a, float b);
/**
 * \brief Maximum float function
 * \param a first float
 * \param b second float
 * \return maximum float
 */
inline float max(float a, float b);
/**
 * \brief Module function
 * This function computes the module of the number given as parameter
 * \param a double parameter
 * \return module result
 */
inline double modulo(double a);

inline int sign_int(int a);

void stampa_bit(char c);
#ifdef __cplusplus
}
#endif

#endif
