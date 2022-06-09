/**
 * @file       task_pos_cmd_handler.c
 * @addtogroup TASK_POS_CMD_HANDLER
 * @{
 */
/*Standard include*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*Chibios include*/
#include "ch.h"
#include "hal.h"

/*Self include*/
#include "task_pos_cmd_handler.h"

/*Module include*/
#include "task_resolver.h"
#include "pos_ctrl.h"

/*Other include*/

/**
 * @brief      Structure of thread resources.
 */
struct runTime{
  thread_t *self;
  binary_semaphore_t pccmdh_bsem;
  float    pos_cmd_user;
  POSC_CMD_HANDLE_T pccmdh;
  bool newCommand;
};

static struct runTime runTime, *pcmdhRuntime;

static THD_WORKING_AREA(waPCMDH, 512);
static THD_FUNCTION(procPCMDH ,p)
{
  /*declare private variable*/
  POSC_CMD_HANDLE_T _privpccmdh = DFLT_INIT_POSC_CMD_HANDLE_T();
  pos_u16t _priv_pos_act_u16 = 0U;
  float _priv_speed_act = 0.0;

  /*Initialization*/
  while(!chThdShouldTerminateX())
  {
    if(MSG_OK == chBSemWaitTimeout(&runTime.pccmdh_bsem, TIME_MS2I(500)))
    {
      /*Semaphore taken, proceed.*/
      /*Update input signal*/
      _privpccmdh.pos_cmd_u16 = POSC_ConvertDeg2U16(runTime.pos_cmd_user);
      _priv_pos_act_u16 = POSC_ConvertDeg2U16(resolver_get_position_deg(0));
      _priv_speed_act = resolver_get_speed(0);

      if(_priv_speed_act > 0.005f)
      {
        /*Rotate in positive direction*/
        _privpccmdh.direction_cmd = POSC_CalcDirection(true, _privpccmdh.pos_cmd_u16, _priv_pos_act_u16, POSC_ALT_DIR_THOLD_U16);
      }
      else if(_priv_speed_act < -0.005f)
      {
        /*Rotate in negative direction*/
        _privpccmdh.direction_cmd = POSC_CalcDirection(false, _privpccmdh.pos_cmd_u16, _priv_pos_act_u16, POSC_ALT_DIR_THOLD_U16);
      }
      else
      {
        /*Stop or very slow at the moment*/
        _privpccmdh.direction_cmd = POSC_CalcDirection(true, _privpccmdh.pos_cmd_u16, _priv_pos_act_u16, POSC_POSU16_180DEG);
      }

      /*Write to runTime*/
      chSysLock();
      memcpy(&runTime.pccmdh, &_privpccmdh, sizeof(POSC_CMD_HANDLE_T));
      chSysUnlock();
      runTime.newCommand = true;
    }
  }
}

/**
 * @brief      This function initialize binary semaphore of postion command processer.
 * 
 * @note       This function is invoke by @p tpcmdh_taskInit() and don't need to be invoke seperately.
 */
void tpcmdh_bsemInit(void)
{ 
  chBSemObjectInit(&runTime.pccmdh_bsem, true);
}

/**
 * @brief      This function initialize the thread for postion command processer.
 * 
 * @note       This function shold be invoke before commend producer thread initialization.
 */
void tpcmdh_taskInit(void)
{
  /*Init runTime*/
  runTime.pos_cmd_user = 0.0f;
  runTime.pccmdh.pos_cmd_u16 = 0U;
  runTime.pccmdh.direction_cmd = true;

  runTime.newCommand = false;
  
  pcmdhRuntime = &runTime;
  
  

  /*Init Semaphore*/
  tpcmdh_bsemInit();

  /*Start dedicated thread*/
  runTime.self = chThdCreateStatic(waPCMDH,sizeof(waPCMDH),NORMALPRIO,procPCMDH,NULL);
}


/*Set and get functions*/
/**
 * @brief      This function set position command and signal the binary semaphore.
 *
 * @param[in]  val   Position command to be set
 */
void tpcmdh_SetPosCmd(float val)
{
  if(isfinite(val))
  {
    if(val > POSC_DEG_MAX)
    {
      runTime.pos_cmd_user = POSC_DEG_MAX;
    }
    else if(val < POSC_DEG_MIN)
    {
      runTime.pos_cmd_user = POSC_DEG_MIN;
    }
    else
    {
      runTime.pos_cmd_user = val;
    }
  }
  chBSemSignal(&runTime.pccmdh_bsem);
}

/**
 * @brief      This function get value of user set position command.
 *
 * @return     Current set value
 */
float tpcmdh_GetPosCmd(void)
{
  return runTime.pos_cmd_user;
}

/**
 * @brief      This function get value @p pos_cmd_u16 produced by position command processer.
 *
 * @return     Current set value
 */
pos_u16t tpcmdh_GetPosCmdU16(void)
{
  return runTime.pccmdh.pos_cmd_u16;
}

/**
 * @brief      This function get value @p direction_cmd produced by position command processer.
 *
 * @return     Current set value
 */
bool tpcmdh_GetDirection(void)
{
  runTime.newCommand = false;
  return runTime.pccmdh.direction_cmd;
}

bool tpcmdh_NewCommand(void)
{
  return runTime.newCommand;
}

/**
 * @}
 */
