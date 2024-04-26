#include "Shoot_app.h"

ShootHandle_t shoot_handle;

void ShootTaskConfig(void)
{
	shoot_handle.console=Console_Pointer();
	
//	shoot_handle.magazine_motor.motor_info= MagazineMotor_Pointer();
	
	shoot_handle.magazine_state = MAGAZINE_INIT_STATE;
//	
//	shoot_handle.magazine_motor.ecd_ratio = MAGAZINE_MOTOR_POSITIVE_DIR * MAGAZINE_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
	
	shoot_handle.trigger_motor.motor_info=TriggerMotor_Pointer();
	
	shoot_handle.trigger_state = TRIGGER_END;
	
	shoot_handle.trigger_motor.ecd_ratio = TRIGGER_MOTOR_POSITIVE_DIR * TRIGGER_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
	
//	pid_init(&shoot_handle.magazine_motor.pid.outer_pid, POSITION_PID, 2000.0f, 500.0f,
//             30.0f, 0.1f, 80.0f);/*µ¯¸Ç*/
//  pid_init(&shoot_handle.magazine_motor.pid.inter_pid, POSITION_PID, 3000.0f, 500.0f,
//             1.0f, 0.0f, 3.0f);
	
	pid_init(&shoot_handle.trigger_motor.pid.outer_pid, POSITION_PID, 300.0f, 60.0f,
             12.0f, 0.0f, 10.5f);/*8£¬0£¬0*/ //8,0,25
  pid_init(&shoot_handle.trigger_motor.pid.inter_pid, POSITION_PID, M2006_MOTOR_MAX_CURRENT, 7000.0f,
             70.0f, 0.0f, 0.0f);/*100£¬0£¬0*/ 
	
	shoot_handle.fric_wheel_motor[0].motor_info=FrictionWheelMotor_1_Pointer();
	shoot_handle.fric_wheel_motor[1].motor_info=FrictionWheelMotor_2_Pointer();
	
	for (uint8_t i = 0; i < 2; i++)/*Ä¦²ÁÂÖ*/
  {
     pid_init(&shoot_handle.fric_wheel_motor[i].pid, POSITION_PID, 10000, 500.0f,
              15.0f, 0.0f, 0.0f);/*9,0,0*/
  }
	
	
}

