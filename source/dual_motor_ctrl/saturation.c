/**
 * @file saturation.c
 * @addtogroup SATURATION
 * @{
 */
/*Standard include*/
#include <stdint.h>
#include <math.h>

/*Self include*/
#include "saturation.h"


/**
 * @brief      Limit input between limit_min and limit_max.<BR>
 *             This function input and output in type @p double.
 *
 * @param[in]  input      Input value.
 * @param[in]  limit_min  Lower limit boundary.
 * @param[in]  limit_max  Upper limit boundary.
 *
 * @return     Value after saturation algorithm.
 */
double SAT_dSat(double input, double limit_min, double limit_max)
{
  double output = 0.0;

  if(input > limit_max)
  {
    output = limit_max;
  }
  else if(input < limit_min)
  {
    output = limit_min;
  }
  else
  {
    output = input;
  }

  return output;
}

/**
 * @brief      Limit input between limit_min and limit_max.<BR>
 *             This function input and output in type @p flaot.
 *
 * @param[in]  input      Input value
 * @param[in]  limit_min  Lower limit boundary.
 * @param[in]  limit_max  Upper limit boundary.
 *
 * @return     Value after saturation algorithm.
 * 
 */
float SAT_fSat(float input, float limit_min, float limit_max)
{
  float output = 0.0f;

  if(input > limit_max)
  {
    output = limit_max;
  }
  else if(input < limit_min)
  {
    output = limit_min;
  }
  else
  {
    output = input;
  }

  return output;
}