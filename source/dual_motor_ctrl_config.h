/**
 * @file       dual_motor_ctrl_config.h
 * @author     Maxie
 * @brief      This header file contains default value of all signals within dual motor controller
 *
 * @addtogroup DUAL_MOTOR_CTRL_CONFIG dual motor control configuration
 * @{
 *
 * @details    
 * @version    { version_number }
 */
#ifndef DUAL_MOTOR_CTRL_CONFIG_H
#define DUAL_MOTOR_CTRL_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif

/*Parameters*/

/*Default valus*/
/*Hardware config*/
#define DMOTC_DFLT_TQ_MOT_MAX_ABS_PC  99.9f
#define DMOTC_DFLT_S_AXIS_MAX_ABS_RPM 5.0f

/*PID config*/  
#define DMOTC_DFLT_P_PID_KP           0.5f
#define DMOTC_DFLT_P_PID_KI           0.0f
#define DMOTC_DFLT_P_PID_KD           0.0f

#define DMOTC_DFLT_S_PID_KP           1.0f
#define DMOTC_DFLT_S_PID_KI           0.025f
#define DMOTC_DFLT_S_PID_KD           0.0f

/*TQBC config*/
#define DMOTC_DFLT_OUT_MAX            10.0f
#define DMOTC_DFLT_OUT_MIN            0.0f
#define DMOTC_DFLT_GAIN               -0.3f
#define DMOTC_DFLT_ZCP                80.0f

/*Position 2 mode config*/
#define DMOTC_DFLT_POS2_THOLD       1.0f
#define DMOTC_DFLT_SPEED            2.0f

#ifdef __cplusplus
}
#endif
#endif



/**
 * @}
 */