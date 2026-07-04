#include "motor_can.h"

//电机传给can1的电流值的数组pid计算完了以后存入这里面
static int16_t motor_can_s_s1[11];
//判断can1有没有超过id为8的电机在初始化时候就已经判断好了
static int8_t motor_can_s_s1_bl;
//电机传给can2的电流值的数组pid计算完了以后存入这里面
static int16_t motor_can_s_s2[11];
//判断can2有没有超过id为8的电机在初始化时候就已经判断好了
static int8_t motor_can_s_s2_bl;

//电机初始化时候*speed_seat这个地址默认指向这里
static int16_t motor_can_speed_seat_init[2][11];

//记录每一个can和id对应在电机结构体的位置方便传入指针和修改模式
 int8_t motor_can_id[2][11];//[0][8]=0

//fp32 pid_id_1[3]={20.0,65,0};  //fp32 pid_id_1[3]={3.2,0.1,0};
//fp32 pid_id_2[3]={5.0,0,0.1};//0.75 0 0.005

//电机的结构体
motor_can_type_def Motor_Can[motor_can_N];

//电机的值的初始化 电机 can id 模式(位置或者速度) 位置环的pid 速度环的pid
static motor_can_init_type_def Motor_can_init[motor_can_N]={
	{{motor_M3508,1, 1,motor_speed},{5.0,0,0.1},{3.2,0.1,0}},
	{{motor_M3508,1, 2,motor_speed},{5.0,0,0.1},{3.2,0.1,0}},
	{{motor_M3508,1, 3,motor_speed},{5.0,0,0.1},{3.2,0.1,0}},

	{{motor_M6020,1, 5,motor_seat },{2.2,0,0.4},{50,40,0}},
	{{motor_M6020,1, 6,motor_seat },{2.5,0,0.3},{50,40,0}},
	{{motor_M6020,1, 7,motor_seat },{2.5,0,0.3},{50,40,0}}

};


static void motor_teyp_init(void);

void motor_can_init(void);

void motor_can_send(void);

/**
  *@brief 		can总线的初始化任务
  *@param[in] 	none
  *@retval		none
**/
void motor_can_init(void){
	
	//电机的初始化包含了电机需要的参数
	motor_teyp_init();
	for(uint8_t i=0;i<motor_can_N;i++){
		
		//判断是不是位置模式
		if(Motor_Can[i].mode==motor_seat){
			//如果是位置模式就先把现在位置存储 避免一开始电机位置归零
			*Motor_Can[i].motor.speed_seat = Motor_Can[i].recv->ecd;
		}
	}

}

/**
  *@brief 		改变电机的*speed_seat的地址指向并并且保留指向前的数据
  *@param[in] 	电机的id
  *@param[in] 	电机的can
  *@param[in]	想要指向的地址 
  *@retval		none
**/
void motor_can_speed(uint8_t id,uint8_t can,int16_t *send){
	
	//记录改变之前的数据
	int16_t motor_can_speed_count = *Motor_Can[motor_can_id[can-1][id-1]].motor.speed_seat;
	
	//改变在"can"下的"id"电机的*speed_seat的地址
	Motor_Can[motor_can_id[can-1][id-1]].motor.speed_seat = send;
	
	//把之前存储的数据传入新地址下
	*send = motor_can_speed_count;
}

/**
  *@brief 		返回位置地址
  *@param[in] 	电机的id
  *@param[in] 	电机的can
  *@param[in]	想要改变的模式
  *@retval		none
**/

const uint16_t *motor_can_return_ecd(uint8_t id,uint8_t can){
	return &Motor_Can[motor_can_id[can-1][id-1]].recv->ecd;
}
const int16_t *motor_can_return_speed(uint8_t id,uint8_t can){
	return &Motor_Can[motor_can_id[can-1][id-1]].recv->speed_rpm;
}

/**
  *@brief 		改变电机的模式
  *@param[in] 	电机的id
  *@param[in] 	电机的can
  *@param[in]	想要改变的模式
  *@retval		none
**/
void motor_can_mode(uint8_t id,uint8_t can,uint8_t mode){
	Motor_Can[motor_can_id[can-1][id-1]].mode = mode;
}

/**
  *@brief 		can总线的电机初始化任务
  *@param[in] 	none
  *@retval		none
**/
static void motor_teyp_init(void){
	
	//can总线的初始化
	can_filter_init();
	
	//can1和can2的id为8以上的判断的初始化
	motor_can_s_s1_bl=0;
	motor_can_s_s2_bl=0;
	
	for(uint8_t i=0;i<motor_can_N;++i){
		
		//把电机名字存入Motor_Can
		Motor_Can[i].motor.model = Motor_can_init[i].motor_init[0];
		
		//把电机的can标识存入Motor_Can
		Motor_Can[i].motor.can = Motor_can_init[i].motor_init[1];
		
		//把电机的id存入Motor_Can
		Motor_Can[i].motor.id = Motor_can_init[i].motor_init[2];
		
		//把电机的模式存入Motor_Can
		Motor_Can[i].mode = Motor_can_init[i].motor_init[3];
		
		//把can和id在Motor_Can中的位置存入motor_can_id中
		motor_can_id[Motor_Can[i].motor.can-1][(Motor_Can[i].motor.id-1)] = i;
		
		//把Motor_Can中的*speed_seat的地址默认指向motor_can_speed_seat_init中
		Motor_Can[i].motor.speed_seat = &motor_can_speed_seat_init[Motor_Can[i].motor.can-1][Motor_Can[i].motor.id-1];
		
		//把Motor_Can中的recv(电机的返回值)的地址获取
		Motor_Can[i].recv = can_motor_measure_point(Motor_Can[i].motor.id-1,Motor_Can[i].motor.can);
		
		//速度环的pid的初始化
		PID_init(&Motor_Can[i].speed,PID_POSITION,Motor_can_init[i].pid_id_speed,16000,10000);
		
		//位置环的pid的初始化
		PID_init(&Motor_Can[i].seat,PID_POSITION_S,Motor_can_init[i].pid_id_seat,16000,16000);
	}
	
	for(uint8_t i=0;i<motor_can_N;++i){
		
		//这一步判断电机id是否有大于8的
		if(Motor_Can[i].motor.id>8){
			//can1的
			if(Motor_Can[i].motor.can == 1)motor_can_s_s1_bl=1;
			//can2的
			else if(Motor_Can[i].motor.can == 2)motor_can_s_s2_bl=1;
		}
		for(uint8_t v=i+1;v<motor_can_N;++v){
			//判断电机是不是有重复定义
			if(Motor_Can[i].motor.id == Motor_Can[v].motor.id){
				if(Motor_Can[i].motor.can == Motor_Can[v].motor.can){
					while(1);//有重复定义就进入死循环
				}
			}
		}
	}
}

/**
  *@brief 		电机的数据发送函数
  *@param[in] 	none
  *@retval		none
**/
static void motor_can_send(void){
	//can1的电机数据发送
	CAN1_cmd_Send(motor_can_s_s1,motor_can_s_s1_bl);

}

/**
  *@brief 		对电机的位置环和速度环进行pid计算
  *@param[in] 	none
  *@retval		none
**/
void motor_can_pid(void){
	for(uint8_t i=0;i<motor_can_N;i++){
		
		//判断电机是否是位置环
		if(Motor_Can[i].mode==motor_seat){
			
			//进行位置环的pid计算
			PID_calc(&Motor_Can[i].seat,Motor_Can[i].recv->ecd,*Motor_Can[i].motor.speed_seat);
			
			//进行速度环的pid计算位置环的输出等于速度的输入
			PID_calc(&Motor_Can[i].speed,Motor_Can[i].recv->speed_rpm,Motor_Can[i].seat.out/10);
			
			//判断电机是不是can1
			if(Motor_Can[i].motor.can == 1){
				motor_can_s_s1[Motor_Can[i].motor.id-1]=Motor_Can[i].speed.out;
			}//判断电机是不是can2
			else if(Motor_Can[i].motor.can == 2){
				motor_can_s_s2[Motor_Can[i].motor.id-1]=Motor_Can[i].speed.out;
			}
		}//判断电机是否是速度环
		else if(Motor_Can[i].mode==motor_speed){
			//进行速度环的pid
			PID_calc(&Motor_Can[i].speed,Motor_Can[i].recv->speed_rpm,*Motor_Can[i].motor.speed_seat);
			//判断电机是不是can1
			if(Motor_Can[i].motor.can == 1){
				motor_can_s_s1[Motor_Can[i].motor.id-1]=Motor_Can[i].speed.out;
			}//判断电机是不是can2
			else if(Motor_Can[i].motor.can == 2){
				motor_can_s_s2[Motor_Can[i].motor.id-1]=Motor_Can[i].speed.out;
			}

		}
	
	}
	//把处理好的结果发送给电机
	motor_can_send();

}
