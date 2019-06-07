#include "gimbal_task.h"
#include "kalman_filter.h"

/* kalman param */
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
	.R_data = {100, 0, 0, 200}   //调整R的值来改变响应速度
};
kalman_filter_t yaw_kalman_filter;
speed_calc_data_t yaw_speed_struct; 
static float yaw_speed_raw;

//初始化kalman参数，把kalman_init放在主函数的init里
void kalman_init(void)
{
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
}

//调用kalman平滑滤波target值
void gimbal_task(void const *argument)
{

  while (1)
  {
				yaw_speed_raw   = target_speed_calc(&yaw_speed_struct,   pgimbal->time, pgimbal->ecd_target_angle.yaw);
				//kalman output  0:angle  1:speed
				float *yaw_kf_result   = kalman_filter_calc(&yaw_kalman_filter,   pgimbal->ecd_target_angle.yaw,   yaw_speed_raw);
				pgimbal->ecd_target_angle.yaw=yaw_kf_result[0];//+yaw_kf_result[1]*50.0f;
	}

}

//通过前后时刻的position和time计算速度
float speed_threshold = 10.0f;
float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position)
{
  S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 1000;
#if 1
    if ((S->speed - S->processed_speed) < -speed_threshold)
    {
        S->processed_speed = S->processed_speed - speed_threshold;
    }
    else if ((S->speed - S->processed_speed) > speed_threshold)
    {
        S->processed_speed = S->processed_speed + speed_threshold;
    }
    else 
#endif
      S->processed_speed = S->speed;
    
    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }
  
  if(S->delay_cnt > 200) // delay 200ms speed = 0
  {
    S->processed_speed = 0;
  }

  return S->processed_speed;
}
