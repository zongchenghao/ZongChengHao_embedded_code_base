#include "Chassis_steer_function.h"
#include "arm_math.h"
#include "math.h"
#include "user_lib.h"

fp32 Chassis_Steer_PID_Calc(Chassis_steer_pid_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb)   //底盘舵电机PID（角度+速度）
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);//角度环
    pid->speed_ref = pid->outer_pid.out;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);//速度环
    return pid->inter_pid.out;
}

void Chassis_MoveTransform(ChassisHandle_t* chassis_handle, fp32* chassis_vx, fp32* chassis_vy)  //底盘云台跟随解算
{
    static fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

    sin_yaw = arm_sin_f32(chassis_handle->gimbal_yaw_ecd_angle / RADIAN_COEF);//RADIAN_COEF(57.3f)，用于将度转换为弧度
    cos_yaw = arm_cos_f32(chassis_handle->gimbal_yaw_ecd_angle / RADIAN_COEF);

    *chassis_vx = cos_yaw * chassis_handle->vx + sin_yaw * chassis_handle->vy;
    *chassis_vy =-sin_yaw * chassis_handle->vx + cos_yaw * chassis_handle->vy;
}

void Steer_Speed_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw) //舵轮轮速解算
{
    float theta = atan(1.0/1.0);                     //程序换算角度为弧度不能直接使用！！
    fp32 steer_vw=chassis_vw*3.14/180;
    float wheel_rpm_ratio;
	
		fp32 wheel_rpm[4];
    fp32 max = 0;
	
	/*M3508_REDUCTION_RATIO为M3508减速比（1.0/19.0）
		*/
	wheel_rpm_ratio = 60.0f/(chassis_handle->structure.wheel_perimeter * M3508_REDUCTION_RATIO);    //舵轮转速转换
	
	wheel_rpm[0]
			  = sqrt(pow(chassis_vx - steer_vw*RADIUS*sin(theta),2)                
			  + pow(chassis_vy - steer_vw*RADIUS*cos(theta),2))* 
				wheel_rpm_ratio*chassis_handle->steer_set[0].speed_direction;        //对应华南理工公式  Vx1-Vw1sin45*RADIUS    
			
			wheel_rpm[1]
			= sqrt(	pow(chassis_vx - steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy + steer_vw*RADIUS*cos(theta),2))* 
	          wheel_rpm_ratio*chassis_handle->steer_set[1].speed_direction;      	 //对应华南理工公式  Vx2+Vw2sin45*RADIUS  																																																																								
			
			wheel_rpm[2]
			= sqrt(	pow(chassis_vx + steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy + steer_vw*RADIUS*cos(theta),2))* 
						wheel_rpm_ratio*chassis_handle->steer_set[2].speed_direction;        //对应华南理工公式  Vy1-Vw1cos45*RADIUS  
																																																																																
			wheel_rpm[3]
			= sqrt(	pow(chassis_vx + steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy - steer_vw*RADIUS*cos(theta),2))*
						wheel_rpm_ratio*chassis_handle->steer_set[3].speed_direction;        //对应华南理工公式  Vy2+Vw2cos45*RADIUS 
	
			//find max item
			for (uint8_t i = 0; i < 4; i++)
			{
				
					if(chassis_handle->turnFlag[i]==1)
					{
						wheel_rpm[i] = -wheel_rpm[i];
					}
					else
					{
						wheel_rpm[i] = wheel_rpm[i];
					}
					
					if (fabs(wheel_rpm[i]) > max)
					{
							max = fabs(wheel_rpm[i]);
					}
			}
			
			memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(fp32));
		}

void Steer_angle_change(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)  
{
	float theta = atan(1.0/1.0);
	fp32 wheel_angle[4];      
	fp32 steer_vw=chassis_vw*3.14/180;
	if((chassis_vx==0)&&(chassis_vy==0)&&(chassis_vw==0))              
	{
		memcpy(wheel_angle, chassis_handle->lastSteeringAngletarget, 4 * sizeof(fp32));
	}
	else      //舵轮角度解算       
	{
		wheel_angle[0]=atan2((chassis_vy-steer_vw*RADIUS*sin(theta)),
							(chassis_vx-steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy1-vw1cos45)/(vx1-vw1sin45))
		wheel_angle[1]=atan2((chassis_vy+steer_vw*RADIUS*sin(theta)),
							(chassis_vx-steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy2-vw2cos45)/(vx2+vw2sin45)) 
		wheel_angle[2]=atan2((chassis_vy+steer_vw*RADIUS*sin(theta)),
							(chassis_vx+steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy3+vw3cos45)/(vx3+vw3sin45)) 
		wheel_angle[3]=atan2((chassis_vy-steer_vw*RADIUS*sin(theta)),
							(chassis_vx+steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy4+vw4cos45)/(vx4-vw4sin45)) 
			 
		for(uint8_t i=0;i<4;i++)                                                
		{
			if(wheel_angle[i]-chassis_handle->lastSteeringAngletarget[i]>PI/2)
			{
				wheel_angle[i]=fmodf(wheel_angle[i]-PI,2*PI);
				chassis_handle->turnFlag[i]=1;
			}
			else if(wheel_angle[i]-chassis_handle->lastSteeringAngletarget[i]<-PI/2)
			{
				wheel_angle[i]=fmodf(wheel_angle[i]+PI,2*PI);
				chassis_handle->turnFlag[i]=1;
			}
			else
			{
				chassis_handle->turnFlag[i]=0;
			}		
		}

		 }
				 
	memcpy(chassis_handle->lastSteeringAngletarget, wheel_angle, 4 * sizeof(fp32));
	for(uint8_t i=0;i<4;i++)
	{
		wheel_angle[i]=wheel_angle[i]*180/PI;       
	}		 
	 memcpy(chassis_handle->steeringAngleTarget, wheel_angle, 4 * sizeof(fp32));  
}

void Steer_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)   //舵轮底盘解算
{	
	Steer_angle_change(chassis_handle,chassis_vx,chassis_vy,chassis_vw);
  calculateTargetRoundCnt(chassis_handle);
	calculateRoundCnt(chassis_handle);	
	Steer_Speed_Calculate(chassis_handle,chassis_vx,chassis_vy,chassis_vw);
}

void Steer_Chassis_ControlCalc(ChassisHandle_t* chassis_handle)      
{
	  static float chassis_vx = 0.0f, chassis_vy = 0.0f;

    Chassis_MoveTransform(chassis_handle, &chassis_vx, &chassis_vy);
    Steer_Calculate(chassis_handle, chassis_vx, chassis_vy, chassis_handle->vw);
}


