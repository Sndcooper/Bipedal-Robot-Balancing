# Auto-Trim Performance Analysis

Based on the telemetry log from `serial_COM13_20260907_222330.log`, here is a breakdown of how the Auto-Trim is currently behaving and how it utilizes encoder data.

## 1. What the Log Shows
In your 1-minute session, Auto-Trim (`ATE:1`) was active for about 63 seconds.
- **Velocity Swings:** `VEL` oscillated significantly between **-497 and +500 ticks/sec**.
- **Trim Response:** `TRIM` adjusted smoothly between **-0.57° and +0.41°**, eventually settling around **-0.108°**.

![Auto Trim Analysis Plots](/C:/Users/vilas/.gemini/antigravity-ide/brain/b313c317-8c65-46d9-8cdb-81085e1ff2a4/scratch/autotrim_analysis.png)

## 2. How the Current Firmware Uses Encoders
Your suggestion to "fuse with the encoders" is actually very close to what the firmware is currently doing, though it uses a simplified control-loop approach rather than a full Kalman filter or weighted state-estimator.

Right now, the firmware calculates the trim by **integrating the encoder velocity**:
```cpp
// 1. Encoder ticks are converted to a raw velocity
float vel_raw = ((deltaL + deltaR) * 0.5f) / dt;

// 2. The raw velocity is smoothed with a low-pass filter (vel_alpha)
vel_current = vel_alpha * vel_current + (1.0f - vel_alpha) * vel_raw;

// 3. The trim bias integrates the velocity error (target velocity = 0)
trim_bias += Ki_trim * (0.0f - vel_current) * dt;
```
Because the robot's target velocity in this static-balance test is `0`, any steady non-zero velocity means the robot is drifting (rolling away). The `trim_bias` slowly accumulates this drift over time and leans the robot back in the opposite direction to halt the drift.

## 3. Why It Might Feel Suboptimal (And How to Fix It)
While the current logic *does* use the encoders, it has a few limitations that might be causing the behavior you're seeing:

1. **It's Reactive, Not Predictive:** Because it relies purely on integration (`Ki_trim * error * dt`), it only corrects the trim *after* the robot has already built up a steady drift velocity.
2. **Phase Lag / Wobble:** If `Ki_trim` is set too high, the auto-trim will over-correct. The robot drifts forward -> the trim leans it backward -> it shoots backward -> the trim leans it forward. This creates a slow, pendulous oscillation (which is evident in the +/- 500 VEL swings in your log).
3. **No Pitch-Velocity Fusion:** Right now, the PID balances purely on IMU Pitch (`Kp`, `Ki`, `Kd`), and the Auto-Trim purely on Encoder Velocity (`Ki_trim`). They are entirely separate.

### Proposed Improvement: LQR / Cascade Control (Weighted Fusion)
If you want to "fuse with the encoders and use weighted data", the standard robotics approach is a **Cascade Controller** or a **State Estimator (LQR)**:
Instead of treating trim as a slow afterthought, you fuse the IMU and Encoders into a single continuous control law:
`Output = (Kp_pitch * pitch_error) + (Kd_pitch * pitch_rate) + (Kp_vel * velocity_error) + (Ki_vel * distance_error)`

By weighting the encoder velocity directly in the main PID loop (rather than using it just to bump the IMU target angle), the robot can actively brake against wheel rotations before they turn into steady drift.
