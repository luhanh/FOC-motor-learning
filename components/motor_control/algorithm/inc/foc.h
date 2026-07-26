#ifndef FOC_H_
#define FOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "foc_utils.h"
#include "foc_defaults.h"

/* ==================== 枚举定义 ==================== */

/** 运动控制模式 */
typedef enum {
    FOC_CTRL_TORQUE = 0x00,            /* 力矩控制 */
    FOC_CTRL_VELOCITY = 0x01,          /* 速度闭环控制 */
    FOC_CTRL_ANGLE = 0x02,             /* 角度闭环控制（级联 PID） */
    FOC_CTRL_VELOCITY_OPENLOOP = 0x03, /* 速度开环控制 */
    FOC_CTRL_ANGLE_OPENLOOP = 0x04,    /* 角度开环控制 */
    FOC_CTRL_ANGLE_NOCASCADE = 0x05,   /* 角度控制（无速度级联） */
} foc_control_type_t;

/** 力矩控制类型 */
typedef enum {
    FOC_TORQUE_VOLTAGE = 0x00,           /* 电压控制 */
    FOC_TORQUE_DC_CURRENT = 0x01,        /* 直流电流控制 */
    FOC_TORQUE_FOC_CURRENT = 0x02,       /* FOC dq 电流控制 */
    FOC_TORQUE_ESTIMATED_CURRENT = 0x03, /* 估算电流控制（需电机参数） */
} foc_torque_type_t;

/** 电机状态 */
typedef enum {
    FOC_STATUS_UNINITIALIZED = 0x00,
    FOC_STATUS_INITIALIZING  = 0x01,
    FOC_STATUS_UNCALIBRATED  = 0x02,
    FOC_STATUS_CALIBRATING   = 0x03,
    FOC_STATUS_READY         = 0x04,
    FOC_STATUS_ERROR         = 0x08,
    FOC_STATUS_CALIB_FAILED  = 0x0E,
    FOC_STATUS_INIT_FAILED   = 0x0F,
} foc_motor_status_t;

/** 传感器方向 */
typedef enum {
    FOC_DIR_CW  = 1,
    FOC_DIR_CCW = -1,
    FOC_DIR_UNKNOWN = 0,
} foc_direction_t;

/* ==================== 电机参数 ==================== */

typedef struct {
    int pole_pairs;                  /* 极对数 */
    float phase_resistance;          /* 相电阻 (Ω) */
    float kv_rating;                 /* KV 额定值 (rpm/V)，FOC_NOT_SET=未设置 */
    float phase_inductance;          /* 相电感 (H)，向后兼容 */
    foc_dq_t axis_inductance;        /* d/q 轴电感 {Ld, Lq}，FOC_NOT_SET=未设置 */
} foc_motor_params_t;

/* ==================== FOC 控制上下文（对标 FOCMotor 类） ==================== */

typedef struct foc_ctx_t {
    /* ---- 电机物理参数 ---- */
    foc_motor_params_t motor;

    /* ---- 控制模式 ---- */
    foc_control_type_t control_type;       /* 运动控制模式 */
    foc_torque_type_t torque_type;         /* 力矩控制类型 */

    /* ---- 限幅 ---- */
    float voltage_limit;                   /* 电压上限 (V) */
    float velocity_limit;                  /* 速度上限 (rad/s) */
    float current_limit;                   /* 电流上限 (A) */

    /* ---- 目标值 ---- */
    float target;                          /* 目标值（含义取决于控制模式） */

    /* ---- 状态变量 ---- */
    float shaft_angle;                     /* 当前机械角度 (rad) */
    float shaft_velocity;                  /* 当前机械角速度 (rad/s) */
    float shaft_angle_sp;                  /* 角度设定点 (rad) */
    float shaft_velocity_sp;               /* 速度设定点 (rad/s) */
    float electrical_angle;                /* 当前电角度 (rad) */
    float current_sp;                      /* 电流设定点 (A) */

    /* ---- 电压/电流 ---- */
    foc_dq_voltage_t voltage;              /* 当前 dq 电压 */
    foc_dq_current_t current;              /* 当前 dq 电流测量值 */
    float voltage_bemf;                    /* 反电动势估算值 (V) */
    float Ualpha;                          /* α 轴电压 (V) */
    float Ubeta;                           /* β 轴电压 (V) */

    /* ---- 前馈 ---- */
    float feed_forward_velocity;           /* 速度前馈 (rad/s) */
    foc_dq_voltage_t feed_forward_voltage; /* 电压前馈 */
    foc_dq_current_t feed_forward_current; /* 电流前馈 */

    /* ---- 传感器 ---- */
    foc_direction_t sensor_direction;      /* 传感器方向 */
    float sensor_offset;                   /* 传感器零位偏移 (rad) */
    float zero_electric_angle;             /* 电零点偏移 (rad) */
    bool pp_check_result;                  /* 极对数校验结果 */

    /* ---- 校准参数 ---- */
    float voltage_sensor_align;            /* 传感器对齐电压 (V) */
    float velocity_index_search;           /* 索引搜索速度 (rad/s) */

    /* ---- 使能与状态 ---- */
    bool enabled;                          /* 电机是否使能 */
    foc_motor_status_t motor_status;       /* 电机状态 */

    /* ---- PID 控制器 ---- */
    pid_controller_t pid_current_q;        /* q 轴电流 PID */
    pid_controller_t pid_current_d;        /* d 轴电流 PID */
    pid_controller_t pid_velocity;         /* 速度环 PID */
    pid_controller_t pid_angle;            /* 角度环 P */

    /* ---- 低通滤波器 ---- */
    lpf_t lpf_current_q;                   /* q 轴电流低通滤波 */
    lpf_t lpf_current_d;                   /* d 轴电流低通滤波 */
    lpf_t lpf_velocity;                    /* 速度低通滤波 */
    lpf_t lpf_angle;                       /* 角度低通滤波 */

    /* ---- 开环时间戳 ---- */
    int64_t open_loop_timestamp;           /* 开环上次调用时间戳 (us) */

    /* ---- 传感器回调（外部注入） ---- */
    void (*sensor_update)(void);                             /* 传感器更新 */
    float (*sensor_get_angle)(void);                         /* 获取角度 (rad) */
    float (*sensor_get_velocity)(void);                      /* 获取角速度 (rad/s) */
    float (*sensor_get_mechanical_angle)(void);              /* 获取机械角度 (rad) */
    bool (*sensor_needs_search)(void);                       /* 是否需要索引搜索 */

    /* ---- 电流检测回调（外部注入，电流模式必需） ---- */
    void (*current_sense_get_phase)(foc_phase_current_t *out);                   /* 获取相电流 */
    foc_dq_current_t (*current_sense_get_dq)(foc_phase_current_t raw, float el); /* 相电流→dq */
    foc_dq_current_t (*current_sense_get_foc)(float el);                         /* 获取 FOC dq 电流 */
    float (*current_sense_get_dc)(float el);                                     /* 获取直流电流幅值 */
    int (*current_sense_driver_align)(float voltage, bool centered);             /* 电流传感器对齐 */
    bool current_sense_initialized;                                              /* 电流传感器已初始化 */

    /* ---- FOC 电压输出回调（注入 bldc_foc_set_voltage） ---- */
    void (*set_phase_voltage)(float Uq, float Ud, float angle_el);

    /* ---- 延时回调（平台相关） ---- */
    void (*delay_ms)(uint32_t ms);
    void (*delay_us)(uint32_t us);
    int64_t (*get_time_us)(void);         /* 获取当前时间戳 (us) */

    /* ---- 电机使能/禁用回调 ---- */
    void (*driver_enable)(void);
    void (*driver_disable)(void);

    /* ---- 性能统计 ---- */
    uint32_t loopfoc_time_us;              /* 电流环执行时间 (us) */
    uint32_t move_time_us;                 /* 运动环执行时间 (us) */

    /* ---- 内部时间戳 ---- */
    int64_t last_loopfoc_timestamp;
    uint32_t last_loopfoc_time;
    int64_t last_move_timestamp;
    uint32_t last_move_time;
    uint32_t motion_cnt;                   /* 运动环降采样计数器 */

    /* ---- 运动控制降采样 ---- */
    uint32_t motion_downsample;            /* 0=不降采样 */

} foc_context_t;

/* ==================== API 声明 ==================== */

/* ---- 初始化 ---- */
esp_err_t foc_init(foc_context_t *ctx, const foc_motor_params_t *params);
esp_err_t foc_init_foc(foc_context_t *ctx);

/* ---- 控制模式配置 ---- */
void foc_set_control_type(foc_context_t *ctx, foc_control_type_t type);
void foc_set_torque_type(foc_context_t *ctx, foc_torque_type_t type);
void foc_set_target(foc_context_t *ctx, float target);

/* ---- 限幅更新（对标 updateXxxLimit） ---- */
void foc_update_velocity_limit(foc_context_t *ctx, float new_limit);
void foc_update_current_limit(foc_context_t *ctx, float new_limit);
void foc_update_voltage_limit(foc_context_t *ctx, float new_limit);

/* ---- 核心控制（对标 loopFOC / move） ---- */
void foc_loop_foc(foc_context_t *ctx);
void foc_move(foc_context_t *ctx, float new_target);

/* ---- 开环控制 ---- */
float foc_velocity_openloop(foc_context_t *ctx, float target_velocity);
float foc_angle_openloop(foc_context_t *ctx, float target_angle);

/* ---- 传感器计算（对标 shaftAngle / shaftVelocity / electricalAngle） ---- */
float foc_shaft_angle(foc_context_t *ctx);
float foc_shaft_velocity(foc_context_t *ctx);
float foc_electrical_angle(foc_context_t *ctx);

/* ---- 反电动势估算 ---- */
float foc_estimate_bemf(const foc_context_t *ctx, float velocity);

/* ---- 电流控制器自整定 ---- */
int foc_tune_current_controller(foc_context_t *ctx, float bandwidth);

/* ---- 电机参数辨识 ---- */
int foc_characterise_motor(foc_context_t *ctx, float voltage, float correction_factor);

/* ---- 传感器对齐 ---- */
int foc_align_sensor(foc_context_t *ctx);
int foc_align_current_sense(foc_context_t *ctx);
int foc_absolute_zero_search(foc_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H_ */
