#ifndef _TASK_DAC_
#define _TASK_DAC_


void analog_output_task_init(void *config);
int8_t analog_output_set_data(uint8_t channel, int16_t *dacv);
int8_t analog_output_set_voltage(uint8_t channel, float *dacv);
int8_t analog_output_set_corse_gain(uint8_t channel, uint16_t *cg);
int8_t analog_output_set_fine_gain(uint8_t channel, uint16_t *fg);
int8_t analog_output_set_offset(uint8_t channel, uint16_t *offset);
void analog_output_task_stop();
int8_t analot_output_get_raw(uint8_t channel, uint16_t *data);
#endif