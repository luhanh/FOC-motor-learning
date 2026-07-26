#ifndef FOC_DEFAULTS_H_
#define FOC_DEFAULTS_H_

/* ==================== 母线电压 ==================== */

#define FOC_DEF_POWER_SUPPLY 12.0f

/* ==================== 速度 PID 默认参数 ==================== */

#define FOC_DEF_PID_VEL_P      0.5f
#define FOC_DEF_PID_VEL_I      10.0f
#define FOC_DEF_PID_VEL_D      0.0f
#define FOC_DEF_PID_VEL_RAMP   FOC_NOT_SET
#define FOC_DEF_PID_VEL_LIMIT  (FOC_DEF_POWER_SUPPLY)

/* ==================== 电流 PID 默认参数 (ESP32) ==================== */

#define FOC_DEF_PID_CURR_P     3.0f
#define FOC_DEF_PID_CURR_I     300.0f
#define FOC_DEF_PID_CURR_D     0.0f
#define FOC_DEF_PID_CURR_RAMP  0.0f
#define FOC_DEF_PID_CURR_LIMIT (FOC_DEF_POWER_SUPPLY)

/* 电流控制带宽 (Hz) */
#define FOC_DEF_CURR_BANDWIDTH 300.0f
/* 电流滤波器时间常数 */
#define FOC_DEF_CURR_FILTER_TF (1.0f / (FOC_2PI * FOC_DEF_CURR_BANDWIDTH))

/* ==================== 角度 P 默认参数 ==================== */

#define FOC_DEF_P_ANGLE_P      20.0f

/* ==================== 限幅默认值 ==================== */

#define FOC_DEF_VEL_LIM        20.0f
#define FOC_DEF_CURRENT_LIM    2.0f

/* ==================== 传感器对齐默认值 ==================== */

#define FOC_DEF_VOLTAGE_SENSOR_ALIGN     8.0f
#define FOC_DEF_INDEX_SEARCH_VELOCITY    1.0f

/* ==================== 滤波器默认值 ==================== */

#define FOC_DEF_VEL_FILTER_TF   0.005f
#define FOC_DEF_ANGLE_FILTER_TF 0.0f

/* ==================== 运动控制默认值 ==================== */

#define FOC_DEF_MON_DOWNSAMPLE  100
#define FOC_DEF_MOTION_DOWNSAMPLE 0

#endif /* FOC_DEFAULTS_H_ */
