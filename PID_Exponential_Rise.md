# PID Control with Exponential Rise

## Objective

Design a PID (or PI) controller such that the closed-loop response rises
smoothly in an **exponential-like curve**, similar to a **first-order
system**.

------------------------------------------------------------------------

## Methods

### 1. Setpoint Shaping (First-order Filter on Setpoint)

Instead of giving the PID a direct step input, filter the setpoint using
a **first-order filter**:

**Discrete form:**

    r_f[k] = r_f[k-1] + α * (r - r_f[k-1])

Where: - `r_f` = filtered setpoint - `r` = target setpoint -
`α = Ts / τr` - `Ts` = sample time - `τr` = filter time constant
(defines how fast the exponential rise is)

The PID tracks `r_f` instead of `r`.\
This forces the system response to follow an exponential trajectory.

------------------------------------------------------------------------

### 2. Controller Design for First-order Closed-loop

If the plant can be approximated as a first-order system:

    Gp(s) = K / (τp s + 1)

Use a PI controller:

    C(s) = Kc * (1 + 1/(Ti s))

Choose: - `Ti = τp` (pole-zero cancellation) - `Kc = τp / (K * τcl)`

Where: - `τcl` = desired closed-loop time constant

This yields a closed-loop transfer function close to a first-order
exponential rise.

------------------------------------------------------------------------

## Practical Tips

-   Use **anti-windup** if actuator saturation exists.
-   Apply **derivative on measurement** if D-term is needed (less noise
    sensitive).
-   Combine **setpoint shaping** with **PI tuning** for smooth
    exponential rise.
-   Increase `τr` for smoother response, decrease for faster rise.

------------------------------------------------------------------------

## Example Implementation (C, BARR-C Style)

``` c
typedef struct
{
    float kp;
    float ki;
    float ts;
    float integ;
    float r_f;     /* filtered setpoint */
    float alpha;   /* Ts / tau_r */
} pi_ctrl_t;

float pi_update(pi_ctrl_t *ctx, float r, float y)
{
    /* 1. First-order filter on setpoint */
    ctx->r_f += ctx->alpha * (r - ctx->r_f);

    /* 2. Error */
    float e = ctx->r_f - y;

    /* 3. PI control */
    ctx->integ += ctx->ki * ctx->ts * e;
    float u = ctx->kp * e + ctx->integ;

    return u;
}
```

------------------------------------------------------------------------

## Summary

-   To achieve exponential-like rise, the most robust method is to use a
    **first-order filter on the setpoint**.
-   If plant dynamics are simple and well-known, a **PI with pole-zero
    cancellation** can enforce first-order closed-loop response.
-   In practice, combine both for smooth, controlled exponential rise.
