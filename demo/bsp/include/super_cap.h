#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "main.h"
#include "graphics.h"
#define CAP_SET_IN_POWER_CANID 0x601
#define CAP_SET_OUT_VOT_CANID 0x602
#define CAP_SET_OUT_CUR_CANID 0x603

#define CAP_STATE_ERROR_CANID 0x610
#define CAP_GET_IN_DATA_CANID 0x611
#define CAP_GET_OUT_DATA_CANID 0x612


typedef struct
{
	struct
	{
		int16_t input_pow;
		uint16_t input_vot;
		int16_t input_cur;
		int16_t output_pow;
		uint16_t output_vot;
		int16_t output_cur;
		uint16_t state;
		uint16_t error;
	}receive_data;
	uint16_t set_target_power;
	float energy;
	float target_output_pow;
	float target_input_pow;
	float output_vot;
	float output_pow;
	float input_pow;
	float per_energy;   
  
} SuperCap;


#define CAP_SET_POWER(x) (cap.target_power = x)

extern SuperCap cap;









#endif /* __SUPER_CAP_H */
