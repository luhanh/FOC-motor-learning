#include "lowpass_filter.h"
#include "foc_utils.h"

/* ==================== 低通滤波器实现 ==================== */

void lpf_init(lpf_t *lpf, float Tf, float Ts) {
    lpf->Tf = Tf;
    lpf->Ts = Ts;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = 0;
}

float lpf_update(lpf_t *lpf, float x) {
    float dt = lpf->Ts;

    /* 自适应采样时间 */
    if (!foc_isset(dt)) {
        dt = 1e-3f;
    }

    /* 对标 Arduino-FOC: dt 过大时重置 */
    if (dt > 0.3f) {
        lpf->y_prev = x;
        return x;
    }

    /* 对标 Arduino-FOC: dt < 0 时回退 */
    if (dt < 0.0f) {
        dt = 1e-3f;
    }

    /* 一阶低通滤波: y = α·y_prev + (1-α)·x */
    float alpha = lpf->Tf / (lpf->Tf + dt);
    float y = alpha * lpf->y_prev + (1.0f - alpha) * x;

    lpf->y_prev = y;
    return y;
}
