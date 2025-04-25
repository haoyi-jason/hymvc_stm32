/**
 * @file saturation.h
 * @author     Maxie
 * @brief      This file implements methods of saturation.
 *
 * @addtogroup SATURATION Saturation
 * @{
 *
 * @details    This module includes saturation function with different data types.
 * 
 * @note       Functions within this module will not check if input input data is valid hence
 *             "garbage in garbage out" situation may happend.
 *             
 * @version    1.0.0
 */

#ifndef SATURATION_H
#define SATURATION_H
#ifdef __cplusplus
extern "C" {
#endif


/*Function*/
double SAT_dSat(double input, double limit_min, double limit_max);
float SAT_fSat(float input, float limit_min, float limit_max);


#ifdef __cplusplus
}
#endif
#endif

/**
 * @}
 */
