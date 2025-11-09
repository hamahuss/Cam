for now, no kalman filter only guidance


----------------------------fix oscialltion -------------------------
Good — since you confirmed the attitude controller is tracking the setpoints well, the oscillation is coming from the guidance output (noisy/discontinuous q_tilt updates, bad timing, or poor prediction). Below I give a focused, practical checklist of guidance-side fixes you can apply immediately, with precise formulas, sensible default parameters to try, and short code recipes (quaternion + angle filters, gating, prediction, and adaptive deadband). Apply them in this order — each step alone often fixes the problem; combine several for the best result.

High-level reasons for oscillation (short)

noisy / jittery pixel detections → rapidly changing LOS → alternating sign or direction of q_tilt.

latency between Jetson → PX4 makes guidance and inner-loop fight each other (without prediction).

discontinuous quaternion sign flips (±q) or taking the long rotation path.

no smoothing / rate limiting on desired attitude → inner loop perfectly tracks fast flips → visible oscillation.

switching between multiple close local minima (target crossing boresight) causing chattering around deadband.

Immediate fixes (apply in this order)
1) Low-pass / slerp the quaternion (best single fix)

Smooth the transmitted q_tilt with spherical linear interpolation (slerp) to avoid abrupt flips.

Use:

q_smoothed = slerp(q_prev, q_new, alpha)


with alpha = 0.05..0.3 (start 0.1). If you prefer rate-limiting in angle domain, limit change in rotation angle per update:

delta_ang = angle_between(q_prev, q_new)
max_step = max_ang_rate * dt  # e.g. 0.5 rad/s * dt
if delta_ang > max_step:
    q_smoothed = slerp(q_prev, q_new, max_step/delta_ang)
else:
    q_smoothed = q_new


Defaults: max_ang_rate = 0.6 rad/s (≈34°/s), dt = 0.02s (50 Hz) → max_step ≈ 0.012 rad.

Why: prevents the attitude loop from chasing fast toggles and is frame-safe (slerp keeps quaternion on unit sphere).

2) Deadband + hysteresis around boresight

If target near center, avoid tiny corrections.

Compute tilt angle θ = 2*acos(q_new.w).

If θ < deadband → send identity quaternion [1,0,0,0].

Use hysteresis: use two thresholds deadband_enter = 0.5°, deadband_exit = 0.3° (so you don't flip in/out continuously).

Defaults: deadband_enter = 0.0087 rad (0.5°), deadband_exit = 0.0052 rad (0.3°).

3) Use a tracker + predict forward for delay (KF on image angles)

Implement a small 2-state KF per axis (α, α̇) or a joint 4-state KF. Use it to (A) smooth α, (B) estimate α̇, (C) predict α(t+τ_d). Use predicted LOS to compute q_tilt. This removes latency-induced oscillations.

Discrete-time KF (process noise Q, meas noise R) — predict step runs at control rate, update when detector gives measurement.

Prediction:

alpha_pred = alpha_hat + alpha_dot_hat * tau_d


Compute v_c(pred) → s_b(pred) → q_new.

Defaults: run KF at camera rate; if detector slow, run optical-flow to fill in. Start Q small on alpha, larger on alpha_dot.

4) Reject bad / low-confidence detections (gating)

If perception gives confidence or bbox size, ignore updates that disagree with prediction:

Compute innovation ν = z - H x_pred, Mahalanobis d2 = ν^T S^{-1} ν. If d2 > χ²(2,0.997) reject. Also reject if confidence < 0.3 or bbox area small.

This prevents sudden swaps to wrong LOS causing big q_tilt flips.

5) Clamp & normalize quaternion sign and take shortest path

Always normalize and enforce scalar-first sign convention (flip if w < 0) before slerp. Use shortest-path slerp (most libraries do).

if q_new.w < 0: q_new = -q_new
q_new = q_new / ||q_new||


This removes 180° sign jumps.

6) Limit desired angular velocity implicitly (angle-rate limiter)

If you still see fast oscillation, enforce a maximum tilt rate:

Compute desired delta angle Δθ = angle_between(q_prev, q_new). If Δθ/dt > ω_tilt_max then scale interpolation so rate = ω_tilt_max.

Suggest ω_tilt_max = 0.6 .. 1.2 rad/s depending on vehicle.

(Equivalent to rate-limited slerp described earlier.)

7) Blend feedforward to reduce correction effort

If your guidance can estimate image angular rates \dotα, map them to body angular rates ω_pred and include in the desired setpoint:

ω_cmd = 2*K_att*q_tilt_vec + ω_pred


When inner loop gets a feedforward rate, it doesn't have to swing back and forth to chase moving target.

8) Low-pass raw pixel input or use median filter

Before angles, filter pixel centroid:

Running median of last N centroids (N=3..7) reduces outliers.

Or use exponential smoothing: u_f = β u_prev + (1-β) u_meas, β=0.6..0.9.

This prevents high-frequency jitter entering guidance.

9) Adaptive deadband from covariance

If KF gives predicted covariance σ, set deadband = base + k*σ. When perception uncertain, widen deadband to avoid aggressive corrections.

Example: deadband = 0.5° + 4*σ_deg.

10) Hysteresis in enabling/disabling guidance override

If PX4 also commanding attitudes (mission), add a small dwell time before switching guidance on/off. E.g., require stable detection for 3 consecutive frames before allowing full override.

Suggested combined pipeline (practical)

Run detection → get centroid (u,v) and confidence.

Optional: median filter / exp smoothing on (u,v).

KF per axis: update measurement; predict to t_now.

Predict forward α_pred = α_hat + αdot_hat * τ_d.

Build q_new from α_pred → vc → sb → axis-angle → quaternion; normalize & enforce w>=0.

Deadband/hysteresis test: if within, set q_new = identity.

Reject if Mahalanobis or confidence low — instead reuse q_prev.

Compute q_smoothed = rate-limited slerp(prev, new, alpha_cap) where alpha_cap chosen by max_ang_rate*dt.

Optionally compute ω_pred and include as feedforward (if replacing error with rates).

Publish q_smoothed.

Concrete code snippets
Quaternion slerp rate-limit (Python)
import numpy as np
from math import acos, sin

def quat_normalize(q):
    q = np.array(q, dtype=float)
    return q / np.linalg.norm(q)

def quat_dot(q1,q2):
    return np.dot(q1, q2)

def slerp(q1, q2, t):
    q1 = quat_normalize(q1); q2 = quat_normalize(q2)
    dot = quat_dot(q1,q2)
    if dot < 0.0:
        q2 = -q2; dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # linear interp
        res = q1 + t*(q2 - q1)
        return quat_normalize(res)
    theta_0 = acos(dot)
    theta = theta_0 * t
    q3 = q2 - q1*dot
    q3 = q3 / np.linalg.norm(q3)
    return q1*np.cos(theta) + q3*np.sin(theta)

def limited_slerp(prev_q, new_q, dt, max_ang_rate):
    # compute angle between
    prev_q = quat_normalize(prev_q); new_q = quat_normalize(new_q)
    dot = np.dot(prev_q, new_q)
    if dot < 0:
        new_q = -new_q; dot = -dot
    angle = 2*acos(max(min(dot,1.0),-1.0))  # geodesic angle
    max_step = max_ang_rate * dt
    if angle <= max_step + 1e-12:
        return new_q
    else:
        t = max_step / angle
        return slerp(prev_q, new_q, t)

KF for angles (very short skeleton)

Use standard constant-velocity KF per axis. Predict covariance → compute Mahalanobis gating. Many implementations exist; use filterpy or implement small 2×2 KF.

Parameters to try (starting values)

update_rate = 50 Hz

slerp alpha if simple: 0.1, or use max_ang_rate = 0.6 rad/s with limited_slerp.

deadband_enter = 0.5°, deadband_exit = 0.3°

KF process noise Q: q_pos = 1e-5, q_vel = 1e-3 ; meas noise R based on pixel-to-angle (~(0.2°)^2 initial).

Confidence threshold: 0.3–0.4.

Mahalanobis gate: p=0.997 (chi2 ~11.8 for 2 DOF).

Logging & diagnosis

Log these while testing:

u,v raw, u_smoothed, alpha_pred, theta_new, theta_prev, delta_theta/dt, q_new, q_prev, q_sent, confidence, KF_P, dot_alpha.
Look for: jitter in u or alpha causing flips; frequent rejections; large delta_theta spikes.

Final notes — practical order to apply now

Implement limited_slerp (rate-limited quaternion smoothing). Test — often fixes oscillation.

Add deadband + hysteresis to ignore tiny tilts.

Add KF + prediction for τ_d (if oscillation persists).

Add gating by Mahalanobis and confidence.

Add feedforward ω_pred if target moves fast
