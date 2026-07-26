#include "foc.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "foc";

/* ==================== 初始化 ==================== */

esp_err_t foc_init(foc_context_t *ctx, const foc_motor_params_t *params) {
    if (ctx == NULL || params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(foc_context_t));

    /* 电机参数 */
    ctx->motor = *params;

    /* 默认限幅 */
    ctx->voltage_limit = FOC_DEF_PID_VEL_LIMIT;
    ctx->velocity_limit = FOC_DEF_VEL_LIM;
    ctx->current_limit = FOC_DEF_CURRENT_LIM;

    /* 默认控制模式 */
    ctx->control_type = FOC_CTRL_TORQUE;
    ctx->torque_type = FOC_TORQUE_VOLTAGE;

    /* 传感器默认方向未知（由校准确定） */
    ctx->sensor_direction = FOC_DIR_UNKNOWN;
    ctx->sensor_offset = 0.0f;
    ctx->zero_electric_angle = FOC_NOT_SET;
    ctx->pp_check_result = false;

    /* 校准参数 */
    ctx->voltage_sensor_align = FOC_DEF_VOLTAGE_SENSOR_ALIGN;
    ctx->velocity_index_search = FOC_DEF_INDEX_SEARCH_VELOCITY;

    /* 电机状态 */
    ctx->enabled = false;
    ctx->motor_status = FOC_STATUS_UNINITIALIZED;

    /* 初始化 PID */
    pid_init(&ctx->pid_current_q,
             FOC_DEF_PID_CURR_P, FOC_DEF_PID_CURR_I, FOC_DEF_PID_CURR_D,
             FOC_DEF_PID_CURR_RAMP, FOC_DEF_PID_CURR_LIMIT, FOC_NOT_SET);
    pid_init(&ctx->pid_current_d,
             FOC_DEF_PID_CURR_P, FOC_DEF_PID_CURR_I, FOC_DEF_PID_CURR_D,
             FOC_DEF_PID_CURR_RAMP, FOC_DEF_PID_CURR_LIMIT, FOC_NOT_SET);
    pid_init(&ctx->pid_velocity,
             FOC_DEF_PID_VEL_P, FOC_DEF_PID_VEL_I, FOC_DEF_PID_VEL_D,
             FOC_DEF_PID_VEL_RAMP, FOC_DEF_PID_VEL_LIMIT, FOC_NOT_SET);
    pid_init(&ctx->pid_angle,
             FOC_DEF_P_ANGLE_P, 0.0f, 0.0f, 0.0f, FOC_DEF_VEL_LIM,
             FOC_NOT_SET);

    /* 初始化低通滤波器 */
    lpf_init(&ctx->lpf_current_q, FOC_DEF_CURR_FILTER_TF, FOC_NOT_SET);
    lpf_init(&ctx->lpf_current_d, FOC_DEF_CURR_FILTER_TF, FOC_NOT_SET);
    lpf_init(&ctx->lpf_velocity, FOC_DEF_VEL_FILTER_TF, FOC_NOT_SET);
    lpf_init(&ctx->lpf_angle, FOC_DEF_ANGLE_FILTER_TF, FOC_NOT_SET);

    /* 降采样 */
    ctx->motion_downsample = FOC_DEF_MOTION_DOWNSAMPLE;
    ctx->motion_cnt = 0;

    /* 前馈清零 */
    ctx->feed_forward_velocity = 0.0f;
    ctx->feed_forward_voltage.d = 0.0f;
    ctx->feed_forward_voltage.q = 0.0f;
    ctx->feed_forward_current.d = 0.0f;
    ctx->feed_forward_current.q = 0.0f;

    /* 电压/电流清零 */
    ctx->voltage.d = 0.0f;
    ctx->voltage.q = 0.0f;
    ctx->current.d = 0.0f;
    ctx->current.q = 0.0f;
    ctx->current_sp = 0.0f;
    ctx->voltage_bemf = 0.0f;

    /* 目标值清零 */
    ctx->target = 0.0f;

    ESP_LOGI(TAG, "FOC 初始化完成 (pp=%d, R=%.3f Ω)",
             ctx->motor.pole_pairs,
             (double)ctx->motor.phase_resistance);

    return ESP_OK;
}

/* ==================== 控制模式切换 ==================== */

void foc_set_control_type(foc_context_t *ctx, foc_control_type_t type) {
    if (ctx->control_type == type) return;

    switch (type) {
    case FOC_CTRL_ANGLE_NOCASCADE:
        if (ctx->control_type == FOC_CTRL_ANGLE ||
            ctx->control_type == FOC_CTRL_ANGLE_OPENLOOP) break;
        /* fall through */
    case FOC_CTRL_ANGLE:
        if (ctx->control_type == FOC_CTRL_ANGLE_OPENLOOP ||
            ctx->control_type == FOC_CTRL_ANGLE_NOCASCADE) break;
        /* fall through */
    case FOC_CTRL_ANGLE_OPENLOOP:
        if (ctx->control_type == FOC_CTRL_ANGLE ||
            ctx->control_type == FOC_CTRL_ANGLE_NOCASCADE) break;
        ctx->target = ctx->shaft_angle;
        break;
    case FOC_CTRL_VELOCITY:
        if (ctx->control_type == FOC_CTRL_VELOCITY_OPENLOOP) break;
        /* fall through */
    case FOC_CTRL_VELOCITY_OPENLOOP:
        if (ctx->control_type == FOC_CTRL_VELOCITY) break;
        ctx->target = 0.0f;
        break;
    case FOC_CTRL_TORQUE:
        ctx->target = 0.0f;
        break;
    }

    ctx->control_type = type;

    /* 同步限幅 */
    foc_update_velocity_limit(ctx, ctx->velocity_limit);
    foc_update_current_limit(ctx, ctx->current_limit);
    foc_update_voltage_limit(ctx, ctx->voltage_limit);

    ESP_LOGI(TAG, "控制模式切换: %d", type);
}

void foc_set_torque_type(foc_context_t *ctx, foc_torque_type_t type) {
    ctx->torque_type = type;

    if (ctx->torque_type == FOC_TORQUE_VOLTAGE) {
        foc_update_voltage_limit(ctx, ctx->voltage_limit);
    } else {
        foc_update_current_limit(ctx, ctx->current_limit);
    }

    ESP_LOGI(TAG, "力矩控制类型切换: %d", type);
}

void foc_set_target(foc_context_t *ctx, float target) {
    ctx->target = target;
}

/* ==================== 限幅更新 ==================== */

void foc_update_velocity_limit(foc_context_t *ctx, float new_limit) {
    ctx->velocity_limit = new_limit;
    if (ctx->control_type != FOC_CTRL_ANGLE_NOCASCADE) {
        ctx->pid_angle.limit = fabsf(new_limit);
    }
}

void foc_update_current_limit(foc_context_t *ctx, float new_limit) {
    ctx->current_limit = new_limit;
    if (ctx->torque_type != FOC_TORQUE_VOLTAGE) {
        ctx->pid_velocity.limit = new_limit;
        if (ctx->control_type == FOC_CTRL_ANGLE_NOCASCADE) {
            ctx->pid_angle.limit = new_limit;
        }
    }
}

void foc_update_voltage_limit(foc_context_t *ctx, float new_limit) {
    ctx->voltage_limit = new_limit;
    ctx->pid_current_q.limit = new_limit;
    ctx->pid_current_d.limit = new_limit;

    if (ctx->torque_type == FOC_TORQUE_VOLTAGE) {
        ctx->pid_velocity.limit = new_limit;
        if (ctx->control_type == FOC_CTRL_ANGLE_NOCASCADE) {
            ctx->pid_angle.limit = new_limit;
        }
    }
}

/* ==================== 传感器计算 ==================== */

float foc_shaft_angle(foc_context_t *ctx) {
    if (ctx->sensor_get_angle == NULL) {
        return ctx->shaft_angle;
    }
    return ctx->sensor_direction * ctx->sensor_get_angle() - ctx->sensor_offset;
}

float foc_shaft_velocity(foc_context_t *ctx) {
    if (ctx->sensor_get_velocity == NULL) {
        return ctx->shaft_velocity;
    }
    float raw = ctx->sensor_direction * ctx->sensor_get_velocity();
    return lpf_update(&ctx->lpf_velocity, raw);
}

float foc_electrical_angle(foc_context_t *ctx) {
    if (ctx->sensor_get_mechanical_angle == NULL) {
        return ctx->electrical_angle;
    }
    return foc_normalize_angle(
        (float)(ctx->sensor_direction * ctx->motor.pole_pairs) *
            ctx->sensor_get_mechanical_angle() -
        ctx->zero_electric_angle);
}

/* ==================== 反电动势估算 ==================== */

float foc_estimate_bemf(const foc_context_t *ctx, float velocity) {
    if (!foc_isset(ctx->motor.kv_rating)) {
        return 0.0f;
    }
    return velocity / (ctx->motor.kv_rating * FOC_SQRT3) / FOC_RPM2RADS;
}

/* ==================== 开环控制 ==================== */

float foc_velocity_openloop(foc_context_t *ctx, float target_velocity) {
    int64_t now = ctx->get_time_us ? ctx->get_time_us() : 0;
    float Ts = (float)(now - ctx->open_loop_timestamp) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
    ctx->open_loop_timestamp = now;

    ctx->shaft_angle = foc_normalize_angle(
        ctx->shaft_angle + target_velocity * Ts);
    ctx->shaft_velocity = target_velocity;

    return (ctx->torque_type == FOC_TORQUE_VOLTAGE)
               ? ctx->voltage_limit
               : ctx->current_limit;
}

float foc_angle_openloop(foc_context_t *ctx, float target_angle) {
    int64_t now = ctx->get_time_us ? ctx->get_time_us() : 0;
    float Ts = (float)(now - ctx->open_loop_timestamp) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
    ctx->open_loop_timestamp = now;

    if (fabsf(target_angle - ctx->shaft_angle) >
        fabsf(ctx->velocity_limit * Ts)) {
        ctx->shaft_angle += foc_sign(target_angle - ctx->shaft_angle) *
                            fabsf(ctx->velocity_limit) * Ts;
        ctx->shaft_velocity = ctx->velocity_limit;
    } else {
        ctx->shaft_angle = target_angle;
        ctx->shaft_velocity = 0.0f;
    }

    return (ctx->torque_type == FOC_TORQUE_VOLTAGE)
               ? ctx->voltage_limit
               : ctx->current_limit;
}

/* ==================== FOC 电流环（对标 loopFOC） ==================== */

void foc_loop_foc(foc_context_t *ctx) {
    /* 校准中不执行 */
    if (ctx->motor_status == FOC_STATUS_CALIBRATING) return;

    /* 更新传感器（即使未使能也更新，避免丢失旋转计数） */
    if (ctx->sensor_update) ctx->sensor_update();

    /* 未使能不输出 */
    if (!ctx->enabled) return;

    /* 计算电角度 */
    if (ctx->control_type == FOC_CTRL_ANGLE_OPENLOOP ||
        ctx->control_type == FOC_CTRL_VELOCITY_OPENLOOP) {
        ctx->electrical_angle =
            foc_electrical_angle_calc(ctx->shaft_angle, ctx->motor.pole_pairs);
    } else {
        ctx->electrical_angle = foc_electrical_angle(ctx);
    }

    /* 力矩控制 */
    switch (ctx->torque_type) {
    case FOC_TORQUE_VOLTAGE:
        ctx->voltage.q =
            foc_constrain(ctx->current_sp, -ctx->voltage_limit,
                          ctx->voltage_limit) +
            ctx->feed_forward_voltage.q;
        ctx->voltage.d = ctx->feed_forward_voltage.d;
        break;

    case FOC_TORQUE_ESTIMATED_CURRENT:
        if (!foc_isset(ctx->motor.phase_resistance)) return;
        ctx->current_sp =
            foc_constrain(ctx->current_sp, -ctx->current_limit,
                          ctx->current_limit) +
            ctx->feed_forward_current.q;

        if (foc_isset(ctx->motor.kv_rating)) {
            ctx->voltage_bemf = foc_estimate_bemf(ctx, ctx->shaft_velocity);
        }

        ctx->current.q = lpf_update(&ctx->lpf_current_q, ctx->current_sp);
        ctx->voltage.q = ctx->current.q * ctx->motor.phase_resistance +
                         ctx->voltage_bemf;
        ctx->voltage.q =
            foc_constrain(ctx->voltage.q, -ctx->voltage_limit,
                          ctx->voltage_limit) +
            ctx->feed_forward_voltage.q;

        if (foc_isset(ctx->motor.axis_inductance.q)) {
            ctx->voltage.d =
                foc_constrain(
                    -ctx->current_sp * ctx->shaft_velocity *
                        ctx->motor.pole_pairs *
                        ctx->motor.axis_inductance.q,
                    -ctx->voltage_limit, ctx->voltage_limit) +
                ctx->feed_forward_voltage.d;
        } else {
            ctx->voltage.d = ctx->feed_forward_voltage.d;
        }
        break;

    case FOC_TORQUE_DC_CURRENT:
        if (ctx->current_sense_get_dc == NULL) return;
        ctx->current_sp =
            foc_constrain(ctx->current_sp, -ctx->current_limit,
                          ctx->current_limit) +
            ctx->feed_forward_current.q;

        ctx->current.q = ctx->current_sense_get_dc(ctx->electrical_angle);
        ctx->current.q = lpf_update(&ctx->lpf_current_q, ctx->current.q);
        ctx->voltage.q =
            pid_update(&ctx->pid_current_q,
                       ctx->current_sp - ctx->current.q) +
            ctx->feed_forward_voltage.q;

        if (foc_isset(ctx->motor.axis_inductance.q)) {
            ctx->voltage.d =
                foc_constrain(
                    -ctx->current_sp * ctx->shaft_velocity *
                        ctx->motor.pole_pairs *
                        ctx->motor.axis_inductance.q,
                    -ctx->voltage_limit, ctx->voltage_limit) +
                ctx->feed_forward_voltage.d;
        } else {
            ctx->voltage.d = ctx->feed_forward_voltage.d;
        }
        break;

    case FOC_TORQUE_FOC_CURRENT:
        if (ctx->current_sense_get_foc == NULL) return;
        ctx->current_sp =
            foc_constrain(ctx->current_sp, -ctx->current_limit,
                          ctx->current_limit) +
            ctx->feed_forward_current.q;

        ctx->current = ctx->current_sense_get_foc(ctx->electrical_angle);
        ctx->current.q = lpf_update(&ctx->lpf_current_q, ctx->current.q);
        ctx->current.d = lpf_update(&ctx->lpf_current_d, ctx->current.d);

        ctx->voltage.q = pid_update(&ctx->pid_current_q,
                                    ctx->current_sp - ctx->current.q);
        ctx->voltage.d = pid_update(
            &ctx->pid_current_d,
            ctx->feed_forward_current.d - ctx->current.d);

        /* d 轴滞后补偿 */
        if (foc_isset(ctx->motor.axis_inductance.q)) {
            ctx->voltage.d = foc_constrain(
                ctx->voltage.d - ctx->current_sp * ctx->shaft_velocity *
                                     ctx->motor.pole_pairs *
                                     ctx->motor.axis_inductance.q,
                -ctx->voltage_limit, ctx->voltage_limit);
        }
        /* q 轴交叉耦合补偿 */
        if (foc_isset(ctx->motor.axis_inductance.d)) {
            ctx->voltage.q = foc_constrain(
                ctx->voltage.q + ctx->current.d * ctx->shaft_velocity *
                                     ctx->motor.pole_pairs *
                                     ctx->motor.axis_inductance.d,
                -ctx->voltage_limit, ctx->voltage_limit);
        }

        ctx->voltage.q += ctx->feed_forward_voltage.q;
        ctx->voltage.d += ctx->feed_forward_voltage.d;
        break;
    }

    /* 输出 PWM */
    if (ctx->set_phase_voltage) {
        ctx->set_phase_voltage(ctx->voltage.q, ctx->voltage.d,
                               ctx->electrical_angle);
    }
}

/* ==================== 运动控制外环（对标 move） ==================== */

void foc_move(foc_context_t *ctx, float new_target) {
    /* 更新目标 */
    if (foc_isset(new_target)) {
        ctx->target = new_target;
    }

    /* 降采样 */
    if (ctx->motion_downsample > 0) {
        if (ctx->motion_cnt++ < ctx->motion_downsample) return;
        ctx->motion_cnt = 0;
    }

    /* 校准中不执行 */
    if (ctx->motor_status == FOC_STATUS_CALIBRATING) return;

    /* 读取传感器（开环模式已在开环函数内更新） */
    if (ctx->control_type != FOC_CTRL_ANGLE_OPENLOOP &&
        ctx->control_type != FOC_CTRL_VELOCITY_OPENLOOP) {
        ctx->shaft_angle = foc_shaft_angle(ctx);
        ctx->shaft_velocity = foc_shaft_velocity(ctx);
    }

    if (!ctx->enabled) return;

    switch (ctx->control_type) {
    case FOC_CTRL_TORQUE:
        ctx->current_sp = ctx->target;
        break;

    case FOC_CTRL_ANGLE_NOCASCADE:
        ctx->shaft_angle_sp = ctx->target;
        ctx->current_sp = pid_update(
            &ctx->pid_angle,
            ctx->shaft_angle_sp -
                lpf_update(&ctx->lpf_angle, ctx->shaft_angle));
        break;

    case FOC_CTRL_ANGLE:
        ctx->shaft_angle_sp = ctx->target;
        ctx->shaft_velocity_sp =
            ctx->feed_forward_velocity +
            pid_update(
                &ctx->pid_angle,
                ctx->shaft_angle_sp -
                    lpf_update(&ctx->lpf_angle, ctx->shaft_angle));
        ctx->shaft_velocity_sp =
            foc_constrain(ctx->shaft_velocity_sp, -ctx->velocity_limit,
                          ctx->velocity_limit);
        ctx->current_sp = pid_update(
            &ctx->pid_velocity,
            ctx->shaft_velocity_sp - ctx->shaft_velocity);
        break;

    case FOC_CTRL_VELOCITY:
        ctx->shaft_velocity_sp = ctx->target;
        ctx->current_sp = pid_update(
            &ctx->pid_velocity,
            ctx->shaft_velocity_sp - ctx->shaft_velocity);
        break;

    case FOC_CTRL_VELOCITY_OPENLOOP:
        ctx->shaft_velocity_sp = ctx->target;
        ctx->current_sp = foc_velocity_openloop(ctx, ctx->shaft_velocity_sp);
        break;

    case FOC_CTRL_ANGLE_OPENLOOP:
        ctx->shaft_angle_sp = ctx->target;
        ctx->current_sp = foc_angle_openloop(ctx, ctx->shaft_angle_sp);
        break;
    }
}

/* ==================== 电流控制器自整定 ==================== */

int foc_tune_current_controller(foc_context_t *ctx, float bandwidth) {
    if (bandwidth <= 0.0f) {
        ESP_LOGE(TAG, "电流环整定失败: BW <= 0");
        return 1;
    }

    if (!foc_isset(ctx->motor.phase_resistance) ||
        (!foc_isset(ctx->motor.phase_inductance) &&
         !foc_isset(ctx->motor.axis_inductance.q))) {
        ESP_LOGW(TAG, "缺少电机参数，尝试自动辨识");
        if (foc_characterise_motor(ctx, ctx->voltage_sensor_align, 1.0f)) {
            return 3;
        }
    } else if (foc_isset(ctx->motor.phase_inductance) &&
               !foc_isset(ctx->motor.axis_inductance.q)) {
        ctx->motor.axis_inductance.q = ctx->motor.phase_inductance;
        ctx->motor.axis_inductance.d = ctx->motor.phase_inductance;
    }

    ctx->pid_current_q.P = ctx->motor.axis_inductance.q * (FOC_2PI * bandwidth);
    ctx->pid_current_q.I =
        ctx->motor.phase_resistance * (FOC_2PI * bandwidth);
    ctx->pid_current_d.P = ctx->motor.axis_inductance.d * (FOC_2PI * bandwidth);
    ctx->pid_current_d.I =
        ctx->motor.phase_resistance * (FOC_2PI * bandwidth);

    float lpf_tf = 1.0f / (FOC_2PI * bandwidth * 5.0f);
    ctx->lpf_current_d.Tf = lpf_tf;
    ctx->lpf_current_q.Tf = lpf_tf;

    ESP_LOGI(TAG, "电流环整定完成: BW=%.1f Hz, Pq=%.3f, Iq=%.3f",
             (double)bandwidth,
             (double)ctx->pid_current_q.P,
             (double)ctx->pid_current_q.I);
    return 0;
}

/* ==================== 电机参数辨识 ==================== */

int foc_characterise_motor(foc_context_t *ctx, float voltage,
                           float correction_factor) {
    if (!ctx->current_sense_initialized) {
        ESP_LOGE(TAG, "电机辨识失败: 电流传感器未初始化");
        return 1;
    }
    if (voltage <= 0.0f) {
        ESP_LOGE(TAG, "电机辨识失败: 电压 <= 0");
        return 2;
    }
    voltage = foc_constrain(voltage, 0.0f, ctx->voltage_limit);

    if (ctx->set_phase_voltage == NULL || ctx->delay_ms == NULL) {
        ESP_LOGE(TAG, "电机辨识失败: 缺少回调");
        return 5;
    }

    ESP_LOGI(TAG, "测量电阻...");

    float el_angle = foc_electrical_angle(ctx);

    /* 零电压参考 */
    ctx->set_phase_voltage(0, 0, el_angle);
    ctx->delay_ms(500);

    foc_phase_current_t raw_zero;
    ctx->current_sense_get_phase(&raw_zero);
    foc_dq_current_t zero_dq =
        ctx->current_sense_get_dq(raw_zero, el_angle);

    /* 斜坡升压测量电阻 */
    el_angle = foc_electrical_angle(ctx);
    for (int i = 0; i < 100; i++) {
        ctx->set_phase_voltage(0, voltage / 100.0f * (float)i, el_angle);
        ctx->delay_ms(3);
    }
    ctx->delay_ms(10);

    foc_phase_current_t raw_r;
    ctx->current_sense_get_phase(&raw_r);
    foc_dq_current_t r_dq = ctx->current_sense_get_dq(raw_r, el_angle);

    ctx->set_phase_voltage(0, 0, el_angle);

    if (fabsf(r_dq.d - zero_dq.d) < 0.2f) {
        ESP_LOGE(TAG, "电机辨识失败: 电流太低");
        return 3;
    }

    float resistance =
        voltage / (correction_factor * (r_dq.d - zero_dq.d));
    if (resistance <= 0.0f) {
        ESP_LOGE(TAG, "电机辨识失败: 估算 R <= 0");
        return 4;
    }

    ESP_LOGI(TAG, "估算相电阻: %.3f Ω", (double)(2.0f * resistance));

    ctx->motor.phase_resistance = 2.0f * resistance;

    /* 电感测量简化版：固定脉冲法 */
    ESP_LOGI(TAG, "测量电感...");

    el_angle = foc_electrical_angle(ctx);
    el_angle = foc_normalize_angle(el_angle + 0.5f * FOC_PI);

    /* 零电流 */
    ctx->current_sense_get_phase(&raw_zero);
    zero_dq = ctx->current_sense_get_dq(raw_zero, el_angle);

    /* 施加电压脉冲 */
    int64_t t0 = ctx->get_time_us ? ctx->get_time_us() : 0;
    ctx->set_phase_voltage(0, voltage, el_angle);
    ctx->delay_us(200);

    foc_phase_current_t raw_l;
    ctx->current_sense_get_phase(&raw_l);
    int64_t t1 = ctx->get_time_us ? ctx->get_time_us() : 0;

    ctx->set_phase_voltage(0, 0, el_angle);

    foc_dq_current_t l_dq = ctx->current_sense_get_dq(raw_l, el_angle);

    float Lq = 0.0f;
    if (t1 > t0 && (l_dq.d - zero_dq.d) > 0.0f) {
        float dt = (float)(t1 - t0) / 1000000.0f;
        float v_eff = voltage - resistance * (l_dq.d - zero_dq.d);
        if (v_eff > 0.0f) {
            Lq = -resistance * dt /
                 logf(v_eff / voltage) / correction_factor;
        }
    }

    float Ld = Lq; /* 非凸极电机 Ld ≈ Lq */

    ctx->motor.axis_inductance.q = Lq;
    ctx->motor.axis_inductance.d = Ld;
    ctx->motor.phase_inductance = (Ld + Lq) / 2.0f;

    ESP_LOGI(TAG, "估算电感: Ld=%.3f mH, Lq=%.3f mH",
             (double)(Ld * 1000.0f),
             (double)(Lq * 1000.0f));

    if (Ld > Lq) {
        ESP_LOGW(TAG, "Ld > Lq, 可能辨识有误");
    }

    return 0;
}

/* ==================== 传感器对齐 ==================== */

int foc_align_sensor(foc_context_t *ctx) {
    if (ctx->sensor_get_angle == NULL ||
        ctx->set_phase_voltage == NULL ||
        ctx->delay_ms == NULL) {
        return 0;
    }

    ESP_LOGI(TAG, "传感器对齐...");

    /* 索引搜索 */
    if (ctx->sensor_needs_search && ctx->sensor_needs_search()) {
        if (!foc_absolute_zero_search(ctx)) {
            return 0;
        }
    }

    float voltage_align = ctx->voltage_sensor_align;

    /* 检测传感器方向 */
    if (ctx->sensor_direction == FOC_DIR_UNKNOWN) {
        for (int i = 0; i <= 500; i++) {
            float angle = FOC_3PI_2 + FOC_2PI * i / 500.0f;
            ctx->set_phase_voltage(voltage_align, 0, angle);
            ctx->sensor_update();
            ctx->delay_ms(2);
        }
        ctx->sensor_update();
        float mid_angle = ctx->sensor_get_angle();

        for (int i = 500; i >= 0; i--) {
            float angle = FOC_3PI_2 + FOC_2PI * i / 500.0f;
            ctx->set_phase_voltage(voltage_align, 0, angle);
            ctx->sensor_update();
            ctx->delay_ms(2);
        }
        ctx->sensor_update();
        float end_angle = ctx->sensor_get_angle();

        ctx->delay_ms(200);

        /* 处理 0↔2π 回绕：差值应取最短弧长 */
        float moved = fabsf(mid_angle - end_angle);
        if (moved > FOC_PI) {
            moved = FOC_2PI - moved;
        }
        if (moved < FOC_MIN_ANGLE_DETECT_MOVEMENT) {
            ESP_LOGE(TAG, "传感器对齐失败: 未检测到运动");
            return 0;
        } else if (mid_angle < end_angle) {
            ESP_LOGI(TAG, "传感器方向: CCW");
            ctx->sensor_direction = FOC_DIR_CCW;
        } else {
            ESP_LOGI(TAG, "传感器方向: CW");
            ctx->sensor_direction = FOC_DIR_CW;
        }

        ctx->pp_check_result =
            !(fabsf(moved * ctx->motor.pole_pairs - FOC_2PI) > 0.5f);
        if (!ctx->pp_check_result) {
            ESP_LOGW(TAG, "极对数校验失败, 估算 pp: %.1f",
                     (double)(FOC_2PI / moved));
        } else {
            ESP_LOGI(TAG, "极对数校验: OK");
        }
    } else {
        ESP_LOGI(TAG, "跳过方向校准");
    }

    /* 电零点校准 */
    if (!foc_isset(ctx->zero_electric_angle)) {
        ctx->set_phase_voltage(voltage_align, 0, FOC_3PI_2);
        ctx->delay_ms(700);
        ctx->sensor_update();
        ctx->zero_electric_angle = 0.0f;
        ctx->zero_electric_angle = foc_electrical_angle(ctx);
        ctx->delay_ms(20);

        ESP_LOGI(TAG, "电零点: %.4f", (double)ctx->zero_electric_angle);

        ctx->set_phase_voltage(0, 0, 0);
        ctx->delay_ms(200);
    } else {
        ESP_LOGI(TAG, "跳过电零点校准");
    }

    return 1;
}

int foc_align_current_sense(foc_context_t *ctx) {
    if (!ctx->current_sense_initialized ||
        ctx->current_sense_driver_align == NULL) {
        return 1;
    }

    ESP_LOGI(TAG, "电流传感器对齐...");
    return ctx->current_sense_driver_align(ctx->voltage_sensor_align, true);
}

int foc_absolute_zero_search(foc_context_t *ctx) {
    ESP_LOGI(TAG, "索引搜索...");

    float limit_vel = ctx->velocity_limit;
    float limit_volt = ctx->voltage_limit;
    ctx->velocity_limit = ctx->velocity_index_search;
    ctx->voltage_limit = ctx->voltage_sensor_align;
    ctx->shaft_angle = 0.0f;

    while (ctx->sensor_needs_search &&
           ctx->sensor_needs_search() &&
           ctx->shaft_angle < FOC_2PI) {
        foc_angle_openloop(ctx, 1.5f * FOC_2PI);
        ctx->sensor_update();
    }

    if (ctx->set_phase_voltage) {
        ctx->set_phase_voltage(0, 0, 0);
    }

    ctx->velocity_limit = limit_vel;
    ctx->voltage_limit = limit_volt;

    bool found = !(ctx->sensor_needs_search && ctx->sensor_needs_search());
    ESP_LOGI(TAG, "索引搜索: %s", found ? "成功" : "失败");
    return found ? 1 : 0;
}

/* ==================== FOC 初始化（对标 initFOC） ==================== */

esp_err_t foc_init_foc(foc_context_t *ctx) {
    int exit_flag = 1;

    ctx->motor_status = FOC_STATUS_CALIBRATING;

    /* 传感器对齐 */
    if (ctx->sensor_get_angle != NULL) {
        exit_flag = foc_align_sensor(ctx);
        if (ctx->sensor_update) ctx->sensor_update();
        ctx->shaft_angle = foc_shaft_angle(ctx);
    } else {
        ESP_LOGI(TAG, "无传感器");
        if (ctx->control_type != FOC_CTRL_ANGLE_OPENLOOP &&
            ctx->control_type != FOC_CTRL_VELOCITY_OPENLOOP) {
            ESP_LOGE(TAG, "无传感器仅支持开环模式");
            exit_flag = 0;
        }
    }

    /* 电流传感器对齐 */
    if (exit_flag && ctx->current_sense_initialized) {
        exit_flag = foc_align_current_sense(ctx);
    }

    if (exit_flag) {
        ESP_LOGI(TAG, "FOC 初始化完成");
        ctx->motor_status = FOC_STATUS_READY;
    } else {
        ESP_LOGE(TAG, "FOC 初始化失败");
        ctx->motor_status = FOC_STATUS_CALIB_FAILED;
        if (ctx->driver_disable) ctx->driver_disable();
    }

    return (exit_flag) ? ESP_OK : ESP_FAIL;
}
