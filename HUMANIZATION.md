# KMBox Humanization Strategy

## Overview
The humanization system is designed to make synthetic mouse movements look natural while maintaining compatibility with real-world KMBox usage patterns.

## Key Design Principles

### 1. **Movement-Aware Scaling**
Humanization intensity adapts based on movement size (via precomputed LUT):
- **Small movements (0-20px)**: MORE jitter (0.8x–0.7x multiplier)
  - Simulates hand tremor during precise positioning
  - Most visible humanization
- **Medium movements (20-60px)**: Moderate jitter (0.7x–0.3x multiplier)
  - Balanced between precision and speed
- **Large movements (60-110px)**: Reduced jitter (0.3x–0.1x multiplier)
  - Intentional movements have less tremor
- **Very large movements (110+px)**: MINIMAL jitter (0.09x–0.05x multiplier)
  - Fast flicks should feel snappy, not sluggish
  - Tremor is almost fully suppressed

### 2. **Velocity-Based Suppression**
Jitter fades out when movement stops:
- At full speed: 1.0x jitter
- As velocity → 0: jitter → 0.1x
- Prevents "shaky" cursor after movement ends
- Mimics hand settling behavior

Tremor (sine-wave oscillation) also has its own suppression curve:
- Below 2 px/frame: full tremor
- Above 8 px/frame: ~12.5% tremor (subtle residual shake)
- Linear ramp between thresholds

### 3. **Physical Mouse Passthrough**
- Physical mouse input: **NO humanization** (untouched)
- Synthetic injections: Light humanization + 0-1ms latency per sub-step
- Core1 handles physical mouse reports; Core0 handles standalone smooth injection
- When a physical mouse is connected, smooth injection is blended into the physical report on Core1, or sent as standalone reports from Core0 when the mouse is idle
- This separation is critical for natural feel and correct report rates

## Humanization Levels

### OFF Mode
- **Use case**: Maximum performance, bot detection irrelevant
- **Settings**: All humanization disabled (no jitter, no overshoot, no delivery error, no onset delay, no subdivision)
- **Max per frame**: 16 px (fixed)
- **Accumulator clamp**: Unlimited
- **Best for**: Testing, high-speed automation

### LOW Mode (±0.0625px jitter)
- **Use case**: Competitive gaming, fast reactions needed
- **Settings**: Barely perceptible jitter (sensor noise floor), ±1% delivery error, 0-1 frame onset delay
- **Max per frame**: 15-17 px (per-session randomized)
- **Overshoot**: Disabled
- **Accumulator clamp**: ±4px
- **Best for**: FPS games, fast-paced scenarios
- **Example**: 10,000 rapid movements should feel smooth

### MEDIUM Mode (±0.17px jitter) [DEFAULT]
- **Use case**: General gaming, balanced feel
- **Settings**: Matches physical mouse sensor noise, ±2% delivery error, 1-3 frame onset delay
- **Max per frame**: 13-19 px (per-session randomized)
- **Overshoot**: 5% chance on 15-120px moves, max 0.5px
- **Accumulator clamp**: ±3px
- **Best for**: Most games, general automation
- **Example**: Small adjustments look human, flicks stay snappy

### HIGH Mode (±0.33px jitter)
- **Use case**: Maximum human-like behavior
- **Settings**: Upper sensor noise, ±3% delivery error, 2-6 frame onset delay
- **Max per frame**: 10-22 px (per-session randomized)
- **Overshoot**: 10% chance on 15-120px moves, max 1.0px
- **Accumulator clamp**: ±2px (tightest)
- **Best for**: Stealth automation, anti-cheat evasion
- **Example**: All movements have visible micro-variations

## Real-World KMBox Usage Patterns

Based on analysis of actual KMBox implementations (Java, C#, Python):

### Pattern 1: Rapid Small Movements (MOST COMMON)
```python
# From ZCban/kmboxNET demo - 10,000 iterations
for i in range(10000):
    kmNet.move(0, 10)   # Move down
    kmNet.move(0, -10)  # Move up
```
**Humanization Strategy:**
- Small movements get 0.8x jitter multiplier
- Velocity suppression prevents accumulated shake
- 1-2ms latency per movement (simulates mouse polling)
- Result: Smooth, no glitches, slight natural variation

### Pattern 2: Interpolated Movement
```csharp
// From uve192/KMBox.NET
client.MouseMoveAuto(500, 300, 1000); // 1000ms duration
```
**Humanization Strategy:**
- Movement spread over many frames
- Easing curves provide acceleration/deceleration
- Jitter scaled by current position in movement
- Result: Natural-looking curved path

### Pattern 3: Large Flicks
```java
// From OceanTw/KMNet.java
client.mouse().move((short) 200, (short) 200);
```
**Humanization Strategy:**
- Large movements get 0.05x jitter multiplier (minimal)
- Fast execution, no sluggishness
- Result: Snappy, responsive, feels intentional

### Pattern 4: Bezier Curves
```csharp
client.MouseMoveBezier(800, 600, 2000, 200, 100, 600, 500);
```
**Humanization Strategy:**
- Smooth interpolation with control points
- Light jitter along curve (doesn't break path)
- Result: Natural arcing motion

## Technical Implementation

### Movement Subdivision (smooth_injection.c)

The core humanization strategy breaks each injected command into **4-8 smaller
sub-movements** instead of executing it as a single queued operation. This makes
the movement signature far more organic and harder to fingerprint.

**How it works:**
1. An incoming movement (e.g., "move 40px right") is split into 4-8 sub-steps
2. Each sub-step gets a randomized fraction (~25% ±30% jitter) of the total, with a floor of 25% of the equal share
3. Sub-steps are staggered with 1-3 frame delays between them (not parallel)
4. Each sub-step gets independent easing curves and frame timing
5. The final sub-step uses the exact remainder to prevent any drift
6. Overshoot (if triggered) is only applied to the final sub-step
7. Onset jitter delays the first sub-step by 0-6 frames (mode-dependent) to break timing correlation
8. Each sub-step receives a small delivery error (±1-3% of its value) to simulate sensor noise

**Subdivision parameters by humanization level:**
- **OFF**: No subdivision (single queue entry, legacy behavior)
- **LOW**: 4-5 sub-steps, light ratio jitter, 0-1 frame onset delay
- **MEDIUM**: 4-6 sub-steps, moderate ratio jitter, 1-3 frame onset delay
- **HIGH**: 5-8 sub-steps, maximum ratio jitter, 2-6 frame onset delay

**Example: A 40px rightward movement at MEDIUM level might become:**
```
Sub-step 1: +12px over 2 frames (ease-in-out)    [onset: 2 frames]
Sub-step 2:  +8px over 1 frame  (linear)         [delay: +1 frame]
Sub-step 3: +11px over 2 frames (ease-out)        [delay: +3 frames]
Sub-step 4:  +9px over 1 frame  (ease-in-out)    [delay: +2 frames]
```
Total: 40px (exact), but spread over ~12 frames with varied timing.

**Movements < 3px** are not subdivided (too small to split meaningfully).
When the queue is nearly full, subdivision gracefully degrades to fewer
sub-steps or falls back to a single entry.
If the queue overflows, remaining movement is added directly to the
sub-pixel accumulator as a safety fallback.

### Jitter Application (smooth_injection.c)
```c
// Calculate movement magnitude once
int32_t move_px = fp_to_int(move_mag);

// Get current velocity magnitude for suppression
int32_t vel_mag = max(abs(velocity_x_fp), abs(velocity_y_fp));

// Suppress jitter when velocity is very low (movement has ended)
int32_t velocity_scale = SMOOTH_FP_ONE;
if (vel_mag < int_to_fp(1)) {
    // Scale from 1.0x down to 0.1x as velocity approaches 0
    velocity_scale = (vel_mag * 9) / int_to_fp(1) + (SMOOTH_FP_ONE / 10);
}

// Movement-aware scaling from LUT (0.8x for small, 0.05x for large)
int32_t movement_jitter_scale = lut_get_jitter_scale_for_movement(move_px);
int32_t jitter_scale = fp_mul(humanization.jitter_amount_fp, movement_jitter_scale);

// Apply velocity-based suppression (stops jitter when movement ends)
jitter_scale = fp_mul(jitter_scale, velocity_scale);

// Get jitter from precomputed LUT (cycles through patterns)
lut_get_jitter(frames_processed, jitter_scale, &jitter_x, &jitter_y);
dx_fp += jitter_x;
dy_fp += jitter_y;
```

### Latency Simulation
```c
// Add 0-1 frame latency for synthetic movements (simulates natural timing variation)
if (mode != INJECT_MODE_IMMEDIATE) {
    frames = frames + rng_range(0, 1); // Random 0-1 extra frames
}

// Variable max_per_frame adds ±1 pixel variation per sub-step
int32_t max_per_frame_fp = int_to_fp(max_per_frame) + rng_range_fp(-SMOOTH_FP_ONE, SMOOTH_FP_ONE);
```
At 1ms HID task interval (1000Hz polling), each frame ≈ 1ms.
Total latency per sub-step = base frames + 0-1 random + inter-substep delay (1-3 frames).

### Report Count Management (usb_hid.c)

The firmware uses a dual-core architecture to prevent report doubling:

```c
// CORE1: Physical mouse report callback (process_mouse_report_internal)
// Blends smooth injection into the physical report in-place
int8_t smooth_x = 0, smooth_y = 0;
if (smooth_has_pending()) {
    smooth_process_frame(&smooth_x, &smooth_y);
    mx += smooth_x;  // Add to physical movement
    my += smooth_y;
}
// Patches the raw HID report buffer with combined values
```

```c
// CORE0: hid_device_task() — standalone smooth injection
if (!connection_state.mouse_connected) {
    // No physical mouse — Core0 is the only sender, safe to drain everything
    kmbox_get_mouse_report(&buttons, &x, &y, &wheel, &pan);
    if (has_smooth) {
        smooth_process_frame(&smooth_x, &smooth_y);
        x += smooth_x;
        y += smooth_y;
    }
    send_report(...);
} else if (has_smooth) {
    // Physical mouse IS connected — only send standalone smooth injection.
    // Do NOT drain kmbox accumulators (Core1 owns those).
    smooth_process_frame(&smooth_x, &smooth_y);
    if (smooth_x != 0 || smooth_y != 0) {
        send_report(smooth_x, smooth_y);
    }
}
```

**Key invariant**: When a physical mouse is connected, Core1 owns the kmbox
movement accumulators and blends smooth injection into physical reports.
Core0 only sends standalone smooth-only reports when the mouse is idle.

## Testing Strategy

### Test 1: Rapid Small Movements
- **Goal**: 10,000 movements in ~10-20 seconds
- **Success**: >500 commands/sec, <1% errors, smooth motion
- **Failure**: Glitches, accumulating drift, too much shake

### Test 2: Small Precise Movements
- **Goal**: 1-5px movements should look human
- **Success**: Light jitter visible, not robotic straight lines
- **Failure**: Perfect straight lines or excessive shake

### Test 3: Large Fast Flicks
- **Goal**: 100-300px movements feel snappy
- **Success**: Fast execution, minimal jitter, responsive
- **Failure**: Too much smoothing, sluggish feeling

### Test 4: Report Rate Verification
- **Goal**: Physical + synthetic = same report count
- **Success**: HID report rate unchanged when synthetic active
- **Failure**: Report rate doubles (creates double input)

## Tuning Guidelines

### If movements look TOO robotic:
1. Increase jitter amount (LOW → MEDIUM → HIGH)
2. Check LUT jitter patterns are being applied (`g_jitter_x_lut`, `g_jitter_y_lut`)
3. Verify velocity suppression isn't too aggressive (check `vel_mag < int_to_fp(1)` threshold)
4. Check that delivery error is enabled (should be ±1-3%)
5. Ensure subdivision is activating (movements ≥ 3px, queue not full)

### If movements look TOO synthetic/shaky:
1. Decrease jitter amount (HIGH → MEDIUM → LOW)
2. Check accumulator clamping is working (±2-4px depending on mode)
3. Verify velocity suppression is working (jitter should fade when movement stops)
4. Check onset jitter isn't too high (2-6 frames at HIGH may be excessive for some use cases)

### If large movements feel sluggish:
1. Check `g_jitter_scale_by_movement_lut` for large movements (should be ≤0.1x above 100px)
2. Check `g_frame_spread_by_movement_lut` (should be ~0.7x for large movements)
3. Verify velocity-matched frame cap is at 80 (not 255)
4. Consider reducing easing curve intensity (more linear for large moves)

### If small movements feel jerky:
1. Check frame spread for small movements (~1.3x multiplier)
2. Verify easing curves are smooth (ease-in-out for larger sub-steps)
3. May need to reduce jitter slightly
4. Check onset delay — too much delay on 1-3px moves can feel unresponsive

## Current Settings (Optimized for KMBox Patterns)

| Setting | OFF | LOW | MEDIUM | HIGH |
|---|---|---|---|---|
| **Jitter** | 0 | ±0.0625px | ±0.17px | ±0.33px |
| **Jitter scale** | — | 0.8x → 0.05x | 0.8x → 0.05x | 0.8x → 0.05x |
| **Delivery error** | 0% | ±1% | ±2% | ±3% |
| **Overshoot** | 0% | 0% | 5% (0.5px max) | 10% (1.0px max) |
| **Onset delay** | 0 frames | 0-1 frames | 1-3 frames | 2-6 frames |
| **Max/frame** | 16 px | 15-17 px | 13-19 px | 10-22 px |
| **Subdivision** | None | 4-5 steps | 4-6 steps | 5-8 steps |
| **Accum clamp** | Unlimited | ±4 px | ±3 px | ±2 px |
| **Vel-matched cap** | 80 frames | 80 frames | 80 frames | 80 frames |

All movement-aware jitter scaling (0.8x→0.05x) is shared across modes via a
single precomputed LUT. Per-session randomization re-seeds the PRNG from
hardware TRNG on every mode change.

Tuned specifically for:
- Rapid 10,000+ movement sequences
- Small precise adjustments
- Large fast flicks
- Mixed real-world usage

## Compatibility Notes

✅ **Compatible with:**
- OceanTw/KMNet.java patterns (rapid movements)
- uve192/KMBox.NET patterns (interpolated/Bezier)
- ZCban/kmboxNET patterns (10k+ iterations)
- Physical mouse + synthetic blending

❌ **Incompatible with:**
- Perfect straight-line movements (subdivision + jitter break linearity)
- Zero-latency requirements (0-6 frame onset delay + inter-step staggering)
- Sub-pixel precision requirements (jitter up to ±0.33px at HIGH)

## Performance Metrics

- **Command rate**: 1000+ commands/sec
- **HID task interval**: 1ms (1000Hz polling)
- **Latency per sub-step**: 1-4ms (base frames + random + inter-step delay)
- **Total movement latency**: Varies by subdivision (4-8 sub-steps, staggered)
- **Jitter range**: ±0.0625px to ±0.33px (mode-dependent)
- **CPU overhead**: <2% (LUT-based, no runtime division in hot path; RP2350 SMULL for fixed-point multiply)
- **Memory**: ~8.5KB for LUTs (flash/.rodata, zero RAM cost)

## Future Improvements

1. **Adaptive humanization**: Learn from physical mouse patterns
2. **Per-application profiles**: Different settings per game
3. **Bezier path generation**: Generate smooth curves automatically
4. **Fatigue simulation**: Slight drift over time (like real hand)
5. **Context awareness**: More jitter when "aiming", less when "flicking"

### Already Implemented
- ~~**Movement subdivision**: Break single commands into multiple sub-steps~~ ✅
- ~~**Delivery error simulation**: Per-sub-step ±1-3% sensor noise~~ ✅
- ~~**Onset jitter**: Random delay before movement start to break timing fingerprints~~ ✅
- ~~**Per-session randomization**: PRNG reseeded from hardware TRNG on mode change~~ ✅
- ~~**Accumulator clamping**: Mode-dependent limits to prevent drift accumulation~~ ✅
- ~~**Deferred flash save**: 2-second debounce to avoid multicore flash crashes~~ ✅
- ~~**Velocity-matched frame cap**: 80-frame max to prevent unresponsive movements~~ ✅
