# BLDCMotor.c — ESP32 直流无刷电机驱动模块架构说明

## 分层架构

```
┌─────────────────────────────────────────────────────────────────────┐
│  上层控制 (motor_angle_control.c)                                    │
│    ├── 角度闭环 PID 控制                                             │
│    ├── 传感器读取 (AS5600)                                           │
│    └── 调用：bldc_motor_set_foc_voltage(Uq, Ud, angle_el)  ─────────┐│
│                                                                     ││
├─────────────────────────────────────────────────────────────────────┤│
│  本模块 (BLDCMotor.c)                                                ││
│                                                                     ││
│  ┌──────────────────────┐   ┌──────────────┐   ┌─────────────────┐  ││
│  │ FOC 调制层            │   │ 驱动抽象层    │   │ 硬件平台层       │  ││
│  │                      │   │              │   │                 │  ││
│  │ set_foc_voltage()    │   │ set_duty()   │   │ LEDC (PWM)      │  ││
│  │   ↓                  │   │ set_phase_   │   │ GPIO (使能)     │  ││
│  │ s_foc_modulate()     │   │   voltage()  │   │                 │  ││
│  │   ├─ SVPWM           │   │ set_phase_   │   │                 │  ││
│  │   ├─ SinePWM         │   │   state()    │   │                 │  ││
│  │   ├─ Trap 120°       │   │ enable/      │   │                 │  ││
│  │   └─ Trap 150°       │   │ disable()    │   │                 │  ││
│  │         ↓            │   │              │   │                 │  ││
│  │  计算出 Ua,Ub,Uc ────→ set_phase_      │   │                 │  ││
│  │                       voltage()        │   │                 │  ││
│  │                              ↓         │   │                 │  ││
│  │                       电压→占空比转换 ──→ set_duty() ────────→ PWM ││
│  └──────────────────────┘   └──────────────┘   └─────────────────┘  ││
│                                                                     ││
├─────────────────────────────────────────────────────────────────────┤│
│  硬件平台 (ESP32)                                                    ││
│    ├── LEDC 外设 → 三通道 PWM 输出 (U/V/W)                           ││
│    └── GPIO → 三相独立使能引脚 (EN_U/EN_V/EN_W)                      ││
└─────────────────────────────────────────────────────────────────────┘
```

## 数据流（以 SVPWM 调制为例）

```
  上层                          本模块                          硬件
  ────                          ────                           ────
  Uq, Ud, θe
    │
    └──→ set_foc_voltage()
           │
           ├── Uq/Ud 限幅 (±voltage_limit)
           ├── 角度归一化 [0, 2π)
           │
           └──→ s_foc_modulate()
                  │
                  ├── Park⁻¹: dq → αβ
                  │   Uα = Ud·cosθ - Uq·sinθ
                  │   Uβ = Ud·sinθ + Uq·cosθ
                  │
                  ├── Clarke⁻¹: αβ → abc
                  │   Ua = Uα
                  │   Ub = -½Uα + √3/2·Uβ
                  │   Uc = -½Uα - √3/2·Uβ
                  │
                  ├── Midpoint Clamp (SVPWM 零序注入)
                  │   center = Vlim/2 - (Umax+Umin)/2
                  │
                  └──→ set_phase_voltage(Ua, Ub, Uc)
                         │
                         ├── 电压限幅 [0, voltage_limit]
                         ├── 电压→占空比 (V / Vsupply)
                         │
                         └──→ set_duty(d_u, d_v, d_w) ─────────→ LEDC
                                │
                                ├── ledc_set_duty(CHx, duty) ───→ U 相 PWM 引脚
                                ├── ledc_set_duty(CHx, duty) ───→ V 相 PWM 引脚
                                └── ledc_set_duty(CHx, duty) ───→ W 相 PWM 引脚
```

## 模块分解

| 序号 | 区块 | 内容 |
|:--:|------|------|
| 1 | 静态上下文 | `s_initialized`, `s_config`, 调制参数 |
| 2 | 内部辅助宏 | `s_pin_isset`, `s_constrain`, 电平判断 |
| 3 | GPIO 初始化 | 三相使能引脚配置与默认电平设置 |
| 4 | LEDC PWM 初始化 | 定时器 + 三通道配置 |
| 5 | 电压→占空比换算 | V/Vsupply, 限幅 [0,1] → 硬件占空比 |
| 6 | FOC 调制内部辅助 | 换相表, min3/max3, 相位状态设置 |
| 7 | FOC 调制核心实现 | `s_foc_modulate()` 4 种策略切换 |
| 8 | 基础接口实现 | init/deinit/enable/disable/duty/voltage/state |
| 9 | FOC 调制接口实现 | set_modulation/set_centered/set_foc_voltage/工具 |

---

## 主要接口说明

### `bldc_motor_init(config)`
初始化驱动：GPIO 使能引脚 → LEDC 定时器 → 三路 PWM 通道。完成后默认禁用状态，需显式调用 `enable()`。内部自动校验 `voltage_limit ≤ voltage_power_supply`。

### `bldc_motor_deinit()`
反初始化：`disable()` → 停止 LEDC 通道 → 复位 GPIO 引脚。已反初始化时再次调用为安全空操作。

### `bldc_motor_enable()` / `bldc_motor_disable()`
- **enable**: 先拉高 EN 引脚 → 再设 PWM=0（防止不确定输出）
- **disable**: 先 PWM 归零（防感应尖峰）→ 再拉低 EN 引脚
- 三相 EN 引脚独立控制，NC 引脚自动跳过

### `bldc_motor_set_duty(d_u, d_v, d_w)` — 底层：直接写硬件占空比
参数范围: `[0, 2^duty_resolution - 1]`，8-bit 分辨率时为 `0~255`。

### `bldc_motor_set_phase_voltage(Ua, Ub, Uc)` — 中层：电压接口
参数范围: `[0, voltage_limit]` (V)。内部: 电压限幅 → 占空比换算 → `set_duty()`。对标 `BLDCDriver3PWM::setPwm()`。

### `bldc_motor_set_phase_state(su, sv, sw)` — 中层：相位状态控制
独立控制每相 EN 引脚通断。用于梯形波换相 / 六步换向 / 故障保护。对标 `BLDCDriver3PWM::setPhaseState()`。

### `bldc_motor_set_foc_voltage(Uq, Ud, angle_el)` — 上层：FOC 入口
| 参数 | 含义 |
|------|------|
| `Uq` | 交轴电压 (V)，产生转矩 |
| `Ud` | 直轴电压 (V)，通常为 0（Id=0 控制策略） |
| `angle_el` | 电角度 (弧度) |

内部: 限幅 → 角度归一化 → `s_foc_modulate()` → 写硬件。对标 `BLDCMotor::setPhaseVoltage()`。

### `bldc_motor_set_modulation(type)` — 配置：选择调制策略

| 枚举值 | 说明 |
|--------|------|
| `BLDC_MODULATION_SINE_PWM` | 正弦 PWM（简单，力矩较小） |
| `BLDC_MODULATION_SVPWM` | 空间矢量 PWM（**默认**，力矩 +15%） |
| `BLDC_MODULATION_TRAPEZOID_120` | 梯形波 120°（6 步换相，有刷感） |
| `BLDC_MODULATION_TRAPEZOID_150` | 梯形波 150°（12 步换相，平滑过渡） |

### `bldc_motor_set_modulation_centered(flag)` — 配置：居中调制
- `true`: 三相电压偏置到 Vlim/2（**默认**，适用于全桥驱动器）
- `false`: 最低相拉到 0V（适合低侧电流采样场景）

### `bldc_motor_normalize_angle(angle)` — 工具
角度归一化到 [0, 2π) 范围。

### `bldc_motor_electrical_angle(mech, pp)` — 工具
机械角度 → 电角度换算：`angle_el = normalize(angle_mech × pole_pairs)`。

---

## 设计决策

1. **静态单实例**: 模块用 `static` 变量管理状态，仅支持单电机。多电机场景可扩展为句柄模式。

2. **三相独立使能**: 对标 DRV8313 等三相半桥驱动器，每个 EN 引脚可独立控制（用于高阻态换相）。

3. **居中调制默认启用**: 全桥驱动器中，居中调制可最大化电压利用率并减少谐波。

4. **SVPWM Midpoint Clamp**: 相比简单正弦 PWM，母线电压利用率提升约 15%（`Vlim/√3 → Vlim`）。

5. **电压→占空比换算在驱动内完成**: 上层只需关心物理量（电压），不感知 PWM 分辨率、母线电压等硬件细节。

6. **安全启停顺序**: enable 先使能再清零，disable 先清零再失能，防止 MOSFET 半桥直通或感应尖峰。

---

## 典型调用流程

```c
// 1. 配置
bldc_motor_config_t cfg = {
    .pwm_pin_u = 10, .pwm_pin_v = 11, .pwm_pin_w = 12,
    .enable_pin_u = 4, .enable_pin_v = NC, .enable_pin_w = NC,
    .enable_active_high = true,
    .voltage_power_supply = 12.0f, .voltage_limit = 0,
    .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
    .channel_u = LEDC_CHANNEL_0, .channel_v = LEDC_CHANNEL_1,
    .channel_w = LEDC_CHANNEL_2,
    .pwm_freq_hz = 20000, .duty_resolution = LEDC_TIMER_8_BIT,
};

// 2. 初始化 + 使能
bldc_motor_init(&cfg);
bldc_motor_enable();

// 3. 可选：切换调制策略
bldc_motor_set_modulation(BLDC_MODULATION_SVPWM);

// 4. 控制循环（FOC 闭环）
while (1) {
    float mech_angle = sensor_read();
    float el_angle = bldc_motor_electrical_angle(mech_angle, 7);
    float Uq = pid_compute(target, mech_angle);
    bldc_motor_set_foc_voltage(Uq, 0.0f, el_angle);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// 5. 停止
bldc_motor_disable();
bldc_motor_deinit();
```
