/**
 * Mouse Movement Counteraction Tool (macOS)
 * 
 * Captures mouse movements via CGEventTap and sends opposite movements
 * to KMBox via serial, effectively locking the cursor.
 * 
 * Usage:
 *   ./mouse_counteract /dev/tty.usbmodem2101
 * 
 * Options:
 *   -b, --baud RATE    Serial baud rate (default: 2000000)
 *   -g, --gain FLOAT   Counteraction gain 0.0-2.0 (default: 1.0)
 *   -d, --deadzone PX  Ignore movements below this threshold (default: 0)
 *   -v, --verbose      Enable verbose logging
 *   -p, --paused       Start in paused mode
 *   -t, --test MODE    Run test mode: rapid, precise, flicks, sweep, mixed, all
 *   -o, --output DIR   Output directory for trace HTML (default: .)
 *   -h, --help         Show this help
 * 
 * Keyboard Controls:
 *   SPACE   Toggle counteraction on/off
 *   +/-     Adjust gain (±0.1)
 *   [/]     Adjust move step size (±5)
 *   P/?     Show statistics
 *   V       Toggle verbose mode
 *   T       Toggle mouse transform (block physical mouse)
 *   Q/ESC   Quit
 * 
 * Mouse Movement (Direct Injection):
 *   W/↑     Move mouse up
 *   A/←     Move mouse left
 *   S/↓     Move mouse down
 *   D/→     Move mouse right
 *   IJKL    Alternative movement keys
 * 
 * Trace Visualization:
 *   Each test produces a trace_<test>_<timestamp>.html file that shows:
 *   - Blue path: What we COMMANDED the KMBox to do (cumulative deltas)
 *   - Orange path: Where the cursor ACTUALLY went (polled from macOS)
 *   - Pink deviation lines: Gap between commanded and observed at each point
 *   - Speed coloring, jitter analysis, interval histograms
 *   - Pan/zoom/hover for inspection
 * 
 * Build:
 *   clang -o mouse_counteract mouse_counteract.c \
 *         -framework CoreGraphics -framework ApplicationServices -lm
 * 
 * Note: Requires Accessibility permissions in System Preferences >
 *       Security & Privacy > Privacy > Accessibility
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <pthread.h>
#include <mach/mach_time.h>

#include <CoreGraphics/CoreGraphics.h>
#include <ApplicationServices/ApplicationServices.h>

// ============================================================================
// Configuration & State
// ============================================================================

typedef struct {
    char port[256];
    int baudrate;
    float gain;
    int deadzone;
    bool verbose;
    volatile bool running;
    volatile bool active;
    char test_mode[32];
    char output_dir[256];
} config_t;

typedef struct {
    int64_t movements_detected;
    int64_t movements_countered;
    int64_t total_dx;
    int64_t total_dy;
    int64_t packets_sent;
    int64_t errors;
    int64_t responses_ok;
    int64_t responses_err;
    time_t start_time;
    time_t last_response_time;
} stats_t;

static config_t g_config = {
    .baudrate = 2000000,
    .gain = 1.0f,
    .deadzone = 0,
    .verbose = false,
    .running = true,
    .active = true,
    .test_mode = "",
    .output_dir = "."
};

static int g_move_step = 20;
static bool g_transform_enabled = false;

static stats_t g_stats = {0};
static int g_serial_fd = -1;
static CFMachPortRef g_event_tap = NULL;
static CFRunLoopSourceRef g_run_loop_source = NULL;
static pthread_mutex_t g_serial_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_serial_reader_thread;
static pthread_t g_serial_sender_thread;
static volatile bool g_show_live_stats = false;

// Lockless ring buffer for movement commands (callback -> sender thread)
#define MOVE_QUEUE_SIZE 256
#define MOVE_QUEUE_MASK (MOVE_QUEUE_SIZE - 1)
typedef struct {
    int16_t dx;
    int16_t dy;
} move_cmd_t;
static move_cmd_t g_move_queue[MOVE_QUEUE_SIZE];
static volatile uint32_t g_move_queue_head = 0;
static volatile uint32_t g_move_queue_tail = 0;

// ============================================================================
// High-Resolution Timing (Mach absolute time)
// ============================================================================

static mach_timebase_info_data_t g_timebase;

static uint64_t time_us(void) {
    uint64_t t = mach_absolute_time();
    // Avoid overflow: divide first when possible (numer is usually 1 on modern Macs)
    if (g_timebase.numer <= g_timebase.denom) {
        return (t / (g_timebase.denom * 1000ULL)) * g_timebase.numer;
    }
    return (t * g_timebase.numer) / (g_timebase.denom * 1000ULL);
}

// ============================================================================
// Dual-Channel Movement Trace Recording
// ============================================================================
//
// Channel 1 (CMD): Every command we send to the KMBox - records the delta
//   and builds a cumulative position showing where we INTENDED the cursor to go.
//
// Channel 2 (OBS): Independent polling of the actual macOS cursor position
//   via CGEventGetLocation() at ~2kHz, showing where it ACTUALLY went.
//
// After the test, we overlay both paths in an HTML visualization so you can
// see if they diverge, if there's jitter, latency, or missed movements.
//

// Command sample: what we told the KMBox
typedef struct {
    uint64_t time_us;   // Relative to trace start
    int16_t  dx;        // Delta we sent
    int16_t  dy;
    int32_t  cum_x;     // Cumulative position
    int32_t  cum_y;
} trace_cmd_t;

// Observation sample: where the cursor actually is
typedef struct {
    uint64_t time_us;   // Relative to trace start
    double   abs_x;     // Absolute screen coordinates
    double   abs_y;
    double   rel_x;     // Relative to starting position
    double   rel_y;
} trace_obs_t;

#define TRACE_CMD_MAX   200000
#define TRACE_OBS_MAX   500000

typedef struct {
    trace_cmd_t*      cmds;
    volatile uint32_t cmd_count;

    trace_obs_t*      obs;
    volatile uint32_t obs_count;

    volatile bool     recording;
    uint64_t          start_us;
    double            start_abs_x;
    double            start_abs_y;
    char              test_name[64];

    pthread_t         poller_thread;
    volatile bool     poller_running;
    pthread_mutex_t   cmd_mutex;
} trace_state_t;

static trace_state_t g_trace = {0};

void trace_init(void) {
    mach_timebase_info(&g_timebase);
    g_trace.cmds = (trace_cmd_t*)malloc(TRACE_CMD_MAX * sizeof(trace_cmd_t));
    g_trace.obs  = (trace_obs_t*)malloc(TRACE_OBS_MAX * sizeof(trace_obs_t));
    if (!g_trace.cmds || !g_trace.obs) {
        fprintf(stderr, "Error: Failed to allocate trace buffers\n");
        exit(1);
    }
    g_trace.cmd_count = 0;
    g_trace.obs_count = 0;
    g_trace.recording = false;
    g_trace.poller_running = false;
    pthread_mutex_init(&g_trace.cmd_mutex, NULL);
}

void trace_free(void) {
    free(g_trace.cmds);
    free(g_trace.obs);
    pthread_mutex_destroy(&g_trace.cmd_mutex);
}

// Cursor position poller thread - runs at ~2kHz during recording
void* trace_poller_thread(void* arg) {
    (void)arg;
    while (g_trace.poller_running) {
        if (g_trace.recording && g_trace.obs_count < TRACE_OBS_MAX) {
            CGEventRef ev = CGEventCreate(NULL);
            if (ev) {
                CGPoint loc = CGEventGetLocation(ev);
                CFRelease(ev);

                uint32_t idx = g_trace.obs_count;
                g_trace.obs[idx].time_us = time_us() - g_trace.start_us;
                g_trace.obs[idx].abs_x = loc.x;
                g_trace.obs[idx].abs_y = loc.y;
                g_trace.obs[idx].rel_x = loc.x - g_trace.start_abs_x;
                g_trace.obs[idx].rel_y = loc.y - g_trace.start_abs_y;
                __sync_synchronize();
                g_trace.obs_count = idx + 1;
            }
        }
        usleep(500);  // ~2kHz
    }
    return NULL;
}

void trace_start(const char* test_name) {
    // Snapshot current cursor position as the origin for both channels
    CGEventRef ev = CGEventCreate(NULL);
    CGPoint loc = {0, 0};
    if (ev) {
        loc = CGEventGetLocation(ev);
        CFRelease(ev);
    }

    g_trace.cmd_count = 0;
    g_trace.obs_count = 0;
    g_trace.start_us = time_us();
    g_trace.start_abs_x = loc.x;
    g_trace.start_abs_y = loc.y;
    strncpy(g_trace.test_name, test_name, sizeof(g_trace.test_name) - 1);
    g_trace.test_name[sizeof(g_trace.test_name) - 1] = '\0';

    g_trace.poller_running = true;
    g_trace.recording = true;
    pthread_create(&g_trace.poller_thread, NULL, trace_poller_thread, NULL);

    // Wait for poller to capture at least one sample so commands
    // never precede the first observation timestamp
    int wait_ms = 0;
    while (g_trace.obs_count == 0 && wait_ms < 100) {
        usleep(500);
        wait_ms++;
    }

    printf("  [TRACE] Recording started (origin: %.0f, %.0f)\n", loc.x, loc.y);
}

void trace_stop(void) {
    g_trace.recording = false;
    g_trace.poller_running = false;
    pthread_join(g_trace.poller_thread, NULL);
    printf("  [TRACE] Stopped: %u commands, %u observations\n",
           g_trace.cmd_count, g_trace.obs_count);
}

// Record a command we sent to the KMBox
void trace_record_cmd(int16_t dx, int16_t dy) {
    if (!g_trace.recording) return;
    pthread_mutex_lock(&g_trace.cmd_mutex);
    uint32_t idx = g_trace.cmd_count;
    if (idx >= TRACE_CMD_MAX) {
        pthread_mutex_unlock(&g_trace.cmd_mutex);
        return;
    }
    g_trace.cmds[idx].time_us = time_us() - g_trace.start_us;
    g_trace.cmds[idx].dx = dx;
    g_trace.cmds[idx].dy = dy;
    g_trace.cmds[idx].cum_x = (idx > 0) ? g_trace.cmds[idx - 1].cum_x + dx : dx;
    g_trace.cmds[idx].cum_y = (idx > 0) ? g_trace.cmds[idx - 1].cum_y + dy : dy;
    g_trace.cmd_count = idx + 1;
    pthread_mutex_unlock(&g_trace.cmd_mutex);
}

// ============================================================================
// HTML Trace Visualization Generator
// ============================================================================

void trace_write_html(const char* test_name) {
    uint32_t ncmd = g_trace.cmd_count;
    uint32_t nobs = g_trace.obs_count;

    if (ncmd < 2 && nobs < 2) {
        printf("  [TRACE] Not enough data (cmd=%u, obs=%u)\n", ncmd, nobs);
        return;
    }

    // Build filename
    char filename[512];
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", t);
    snprintf(filename, sizeof(filename), "%s/trace_%s_%s.html",
             g_config.output_dir, test_name, ts);

    FILE* f = fopen(filename, "w");
    if (!f) {
        fprintf(stderr, "  [TRACE] Cannot create %s: %s\n", filename, strerror(errno));
        return;
    }

    // ---- Compute bounds ----
    int32_t cMinX = 0, cMaxX = 0, cMinY = 0, cMaxY = 0;
    for (uint32_t i = 0; i < ncmd; i++) {
        if (g_trace.cmds[i].cum_x < cMinX) cMinX = g_trace.cmds[i].cum_x;
        if (g_trace.cmds[i].cum_x > cMaxX) cMaxX = g_trace.cmds[i].cum_x;
        if (g_trace.cmds[i].cum_y < cMinY) cMinY = g_trace.cmds[i].cum_y;
        if (g_trace.cmds[i].cum_y > cMaxY) cMaxY = g_trace.cmds[i].cum_y;
    }
    double oMinX = 0, oMaxX = 0, oMinY = 0, oMaxY = 0;
    for (uint32_t i = 0; i < nobs; i++) {
        if (g_trace.obs[i].rel_x < oMinX) oMinX = g_trace.obs[i].rel_x;
        if (g_trace.obs[i].rel_x > oMaxX) oMaxX = g_trace.obs[i].rel_x;
        if (g_trace.obs[i].rel_y < oMinY) oMinY = g_trace.obs[i].rel_y;
        if (g_trace.obs[i].rel_y > oMaxY) oMaxY = g_trace.obs[i].rel_y;
    }

    double allMinX = fmin((double)cMinX, oMinX);
    double allMaxX = fmax((double)cMaxX, oMaxX);
    double allMinY = fmin((double)cMinY, oMinY);
    double allMaxY = fmax((double)cMaxY, oMaxY);
    double rangeX = allMaxX - allMinX; if (rangeX < 10) rangeX = 10;
    double rangeY = allMaxY - allMinY; if (rangeY < 10) rangeY = 10;

    // ---- Deviation analysis: interpolate obs at each cmd timestamp ----
    // We compare the cumulative commanded position to the interpolated
    // observed position at the same point in time. Commands that fall
    // before the first observation or after the last are skipped to
    // avoid uint64 underflow when subtracting timestamps.
    double devSum = 0, devMax = 0;
    uint32_t devCount = 0;
    uint32_t osi = 0;
    for (uint32_t ci = 0; ci < ncmd && nobs >= 2; ci++) {
        uint64_t ct = g_trace.cmds[ci].time_us;
        // Skip commands before the first observation (poller not started yet)
        if (ct < g_trace.obs[0].time_us) continue;
        // Skip commands after the last observation
        if (ct > g_trace.obs[nobs - 1].time_us) break;
        // Advance observation index to bracket the command timestamp
        while (osi + 1 < nobs && g_trace.obs[osi + 1].time_us <= ct) osi++;
        if (osi + 1 >= nobs) break;
        uint64_t t0 = g_trace.obs[osi].time_us;
        uint64_t t1 = g_trace.obs[osi + 1].time_us;
        if (t1 <= t0 || ct < t0) continue;
        double fr = (double)(ct - t0) / (double)(t1 - t0);
        if (fr < 0.0 || fr > 1.0) continue;  // Sanity guard
        double ox = g_trace.obs[osi].rel_x + fr * (g_trace.obs[osi + 1].rel_x - g_trace.obs[osi].rel_x);
        double oy = g_trace.obs[osi].rel_y + fr * (g_trace.obs[osi + 1].rel_y - g_trace.obs[osi].rel_y);
        double ddx = ox - (double)g_trace.cmds[ci].cum_x;
        double ddy = oy - (double)g_trace.cmds[ci].cum_y;
        double dev = sqrt(ddx * ddx + ddy * ddy);
        devSum += dev;
        if (dev > devMax) devMax = dev;
        devCount++;
    }
    double devAvg = devCount > 0 ? devSum / devCount : 0;

    // ---- Observed path jitter: perpendicular deviation ----
    double jitSum = 0, jitMax = 0;
    uint32_t jitCount = 0;
    for (uint32_t i = 2; i < nobs; i++) {
        double vx = g_trace.obs[i].rel_x - g_trace.obs[i - 2].rel_x;
        double vy = g_trace.obs[i].rel_y - g_trace.obs[i - 2].rel_y;
        double len = sqrt(vx * vx + vy * vy);
        if (len < 0.5) continue;
        double px = g_trace.obs[i - 1].rel_x - g_trace.obs[i - 2].rel_x;
        double py = g_trace.obs[i - 1].rel_y - g_trace.obs[i - 2].rel_y;
        double perp = fabs(vx * py - vy * px) / len;
        jitSum += perp;
        if (perp > jitMax) jitMax = perp;
        jitCount++;
    }
    double jitAvg = jitCount > 0 ? jitSum / jitCount : 0;

    // ---- Comprehensive Humanization Analysis ----
    // The KMBox may add jitter/smoothing to our commands. We analyze the
    // OBSERVED cursor path for telltale signs of humanization vs robotic movement.

    // 1. Observed delta analysis: variance in per-sample movement
    double obsTotalDist = 0;
    uint32_t obsMoving = 0;  // Samples where cursor actually moved
    double obsMagSum = 0, obsMagSqSum = 0;
    uint32_t obsDirFlipX = 0, obsDirFlipY = 0;
    int prevSgnX = 0, prevSgnY = 0;
    uint32_t obsSubPx = 0;  // Sub-pixel micro-movements (humanization artifact)

    for (uint32_t i = 1; i < nobs; i++) {
        double dx = g_trace.obs[i].rel_x - g_trace.obs[i - 1].rel_x;
        double dy = g_trace.obs[i].rel_y - g_trace.obs[i - 1].rel_y;
        double mag = sqrt(dx * dx + dy * dy);

        if (mag > 0.01) {
            obsTotalDist += mag;
            obsMoving++;
            obsMagSum += mag;
            obsMagSqSum += mag * mag;

            // Track direction reversals (sign flips in X or Y)
            int sX = (dx > 0.05) ? 1 : (dx < -0.05) ? -1 : 0;
            int sY = (dy > 0.05) ? 1 : (dy < -0.05) ? -1 : 0;
            if (sX != 0 && prevSgnX != 0 && sX != prevSgnX) obsDirFlipX++;
            if (sY != 0 && prevSgnY != 0 && sY != prevSgnY) obsDirFlipY++;
            if (sX) prevSgnX = sX;
            if (sY) prevSgnY = sY;

            // Sub-pixel movements: cursor moved but < 0.5px
            if (mag < 0.5) obsSubPx++;
        }
    }

    // Path efficiency: displacement / total distance (1.0 = perfectly straight)
    double dispX = (nobs > 1) ? g_trace.obs[nobs - 1].rel_x - g_trace.obs[0].rel_x : 0;
    double dispY = (nobs > 1) ? g_trace.obs[nobs - 1].rel_y - g_trace.obs[0].rel_y : 0;
    double displacement = sqrt(dispX * dispX + dispY * dispY);
    double pathEff = (obsTotalDist > 1) ? displacement / obsTotalDist : 1.0;

    // Direction reversal rate (per 100 moving samples)
    double dirFlipRate = (obsMoving > 10) ?
        (double)(obsDirFlipX + obsDirFlipY) / obsMoving * 100.0 : 0;

    // Speed coefficient of variation (robotic = near 0, humanized = higher)
    double magMean = (obsMoving > 0) ? obsMagSum / obsMoving : 0;
    double magVar = (obsMoving > 1) ?
        (obsMagSqSum / obsMoving - magMean * magMean) : 0;
    if (magVar < 0) magVar = 0;  // Numerical guard
    double speedCV = (magMean > 0.01) ? sqrt(magVar) / magMean : 0;

    // Sub-pixel noise percentage
    double subPxPct = (nobs > 1) ? (double)obsSubPx / (nobs - 1) * 100.0 : 0;

    // 2. Command timing regularity analysis
    double intSum = 0, intSqSum = 0;
    if (ncmd > 2) {
        for (uint32_t i = 1; i < ncmd; i++) {
            double iv = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i - 1].time_us);
            intSum += iv;
            intSqSum += iv * iv;
        }
    }
    double intMean = (ncmd > 2) ? intSum / (ncmd - 1) : 0;
    double intVar = (ncmd > 2) ? (intSqSum / (ncmd - 1) - intMean * intMean) : 0;
    if (intVar < 0) intVar = 0;
    double intCV = (intMean > 0) ? sqrt(intVar) / intMean : 0;

    // 3. Command delta repetition (consecutive identical dx,dy = robotic input)
    uint32_t cmdRepeats = 0;
    for (uint32_t i = 1; i < ncmd; i++) {
        if (g_trace.cmds[i].dx == g_trace.cmds[i - 1].dx &&
            g_trace.cmds[i].dy == g_trace.cmds[i - 1].dy)
            cmdRepeats++;
    }
    double cmdRepPct = (ncmd > 1) ? (double)cmdRepeats / (ncmd - 1) * 100.0 : 0;

    // 4. Perpendicular scatter on straight segments of the observed path
    //    Look at consecutive windows of 10 obs samples. If the commanded
    //    direction was constant during that window, measure how much the
    //    observed path wobbles perpendicular to the travel direction.
    double perpScatterSum = 0;
    uint32_t perpScatterN = 0;
    for (uint32_t i = 10; i < nobs; i += 5) {
        // Direction vector over window
        double wx = g_trace.obs[i].rel_x - g_trace.obs[i - 10].rel_x;
        double wy = g_trace.obs[i].rel_y - g_trace.obs[i - 10].rel_y;
        double wlen = sqrt(wx * wx + wy * wy);
        if (wlen < 2.0) continue;  // Skip stationary windows
        // Perpendicular unit vector
        double px = -wy / wlen, py = wx / wlen;
        // Measure scatter of intermediate points from the line
        for (uint32_t j = i - 9; j < i; j++) {
            double dx = g_trace.obs[j].rel_x - g_trace.obs[i - 10].rel_x;
            double dy = g_trace.obs[j].rel_y - g_trace.obs[i - 10].rel_y;
            double perpDist = fabs(dx * px + dy * py);
            perpScatterSum += perpDist;
            perpScatterN++;
        }
    }
    double perpScatter = (perpScatterN > 0) ? perpScatterSum / perpScatterN : 0;

    // 5. Acceleration smoothness: how jerkily does the cursor speed change?
    //    Lower = smoother = more robotic. Humanized movement has micro-accelerations.
    double accelSum = 0;
    uint32_t accelN = 0;
    double prevSpeed = 0;
    for (uint32_t i = 1; i < nobs; i++) {
        double dx = g_trace.obs[i].rel_x - g_trace.obs[i - 1].rel_x;
        double dy = g_trace.obs[i].rel_y - g_trace.obs[i - 1].rel_y;
        double dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i - 1].time_us);
        if (dt < 1) continue;
        double spd = sqrt(dx * dx + dy * dy) / dt;
        if (i > 1 && prevSpeed > 0.001) {
            double accel = fabs(spd - prevSpeed) / (dt / 1000.0);  // px/ms²
            accelSum += accel;
            accelN++;
        }
        prevSpeed = spd;
    }
    double accelJerk = (accelN > 0) ? accelSum / accelN : 0;

    // ---- Humanization Score: 0-100 weighted composite ----
    // Each factor contributes points. Higher = more humanized.
    double hScore = 0;
    // Perpendicular scatter on straight segments (most reliable indicator)
    // 0px = robotic, 0.3-1.5px = good humanization
    if (perpScatter > 0.05) hScore += fmin(perpScatter / 0.5, 1.0) * 25;

    // Observed jitter (perpendicular deviation from local line)
    if (jitAvg >= 0.1) hScore += fmin(jitAvg / 0.6, 1.0) * 20;

    // Direction reversals indicate micro-corrections/tremor
    if (dirFlipRate > 2) hScore += fmin(dirFlipRate / 15.0, 1.0) * 15;

    // Speed variance: constant speed = robotic
    if (speedCV > 0.05) hScore += fmin(speedCV / 0.25, 1.0) * 15;

    // Sub-pixel noise: fractional movements suggest interpolation
    if (subPxPct > 1) hScore += fmin(subPxPct / 8.0, 1.0) * 10;

    // Acceleration jerk: micro-accelerations
    if (accelJerk > 0.001) hScore += fmin(accelJerk / 0.05, 1.0) * 10;

    // Path inefficiency: not perfectly straight = good
    if (pathEff < 0.995) hScore += fmin((1.0 - pathEff) / 0.05, 1.0) * 5;

    if (hScore > 100) hScore = 100;

    const char* hGrade, *hCls;
    if      (hScore < 10) { hGrade = "Robotic";   hCls = "b"; }
    else if (hScore < 25) { hGrade = "Minimal";   hCls = "w"; }
    else if (hScore < 50) { hGrade = "Moderate";  hCls = "w"; }
    else if (hScore < 75) { hGrade = "Good";      hCls = "g"; }
    else                  { hGrade = "Excellent";  hCls = "g"; }

    uint64_t totalUs = 0;
    if (ncmd > 0) totalUs = g_trace.cmds[ncmd - 1].time_us;
    if (nobs > 0 && g_trace.obs[nobs - 1].time_us > totalUs)
        totalUs = g_trace.obs[nobs - 1].time_us;

    // Command interval histogram
    uint32_t cmdHist[8] = {0};
    for (uint32_t i = 1; i < ncmd; i++) {
        double ms = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us) / 1000.0;
        int b = (ms < 1) ? 0 : (ms < 2) ? 1 : (ms < 5) ? 2 : (ms < 10) ? 3 :
                (ms < 20) ? 4 : (ms < 50) ? 5 : (ms < 100) ? 6 : 7;
        cmdHist[b]++;
    }

    // ---- Write HTML ----
    fprintf(f,
        "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n"
        "<meta charset=\"UTF-8\">\n"
        "<title>KMBox Trace: %s</title>\n"
        "<style>\n"
        "* { margin:0; padding:0; box-sizing:border-box; }\n"
        "body { background:#0a0a0f; color:#c0c0c8; font-family:'SF Mono','Fira Code','Consolas',monospace; font-size:12px; }\n"
        ".hdr { padding:14px 20px; border-bottom:1px solid #1a1a24; display:flex; justify-content:space-between; align-items:center; }\n"
        ".hdr h1 { font-size:15px; color:#e0e0e8; font-weight:600; }\n"
        ".hdr .sub { color:#555; }\n"
        ".main { display:flex; height:calc(100vh - 48px); }\n"
        ".cvs { flex:1; position:relative; background:#0c0c14; cursor:crosshair; }\n"
        "canvas { display:block; width:100%%; height:100%%; }\n"
        ".sb { width:330px; padding:12px; overflow-y:auto; border-left:1px solid #1a1a24; }\n"
        ".sg { margin-bottom:14px; }\n"
        ".sg h3 { font-size:10px; text-transform:uppercase; letter-spacing:1.2px; color:#444; margin-bottom:5px; }\n"
        ".sr { display:flex; justify-content:space-between; padding:2px 0; }\n"
        ".sr .l { color:#777; } .sr .v { color:#ccc; }\n"
        ".sr .v.g { color:#4ec970; } .sr .v.w { color:#e8a838; } .sr .v.b { color:#e84848; }\n"
        ".hb { display:flex; align-items:center; margin:1px 0; }\n"
        ".hb .hl { width:55px; text-align:right; padding-right:6px; color:#555; font-size:10px; }\n"
        ".hb .bar { height:10px; background:#2a5a9a; border-radius:2px; min-width:1px; }\n"
        ".hb .hc { padding-left:5px; color:#444; font-size:10px; }\n"
        ".ctrl { margin-top:12px; padding-top:12px; border-top:1px solid #1a1a24; }\n"
        ".ctrl label { display:flex; align-items:center; gap:6px; margin:3px 0; cursor:pointer; color:#777; }\n"
        ".ctrl input[type=checkbox] { accent-color:#4ec970; }\n"
        ".ctrl input[type=range] { width:100px; accent-color:#4e88c9; }\n"
        ".leg { display:flex; gap:14px; flex-wrap:wrap; margin-top:10px; }\n"
        ".leg-i { display:flex; align-items:center; gap:5px; }\n"
        ".leg-s { width:18px; height:3px; border-radius:1px; }\n"
        ".tip { position:absolute; pointer-events:none; background:rgba(0,0,0,0.88); border:1px solid #333; "
        "border-radius:4px; padding:6px 10px; font-size:11px; color:#ccc; display:none; z-index:10; white-space:nowrap; }\n"
        "</style>\n</head>\n<body>\n",
        test_name);

    // Header
    fprintf(f,
        "<div class=\"hdr\">\n"
        "  <h1>KMBox Trace &mdash; %s</h1>\n"
        "  <span class=\"sub\">%u cmds &middot; %u obs &middot; %.1fms &middot; %s</span>\n"
        "</div>\n",
        test_name, ncmd, nobs, (double)totalUs / 1000.0, ts);

    fprintf(f, "<div class=\"main\">\n"
               "<div class=\"cvs\"><canvas id=\"c\"></canvas><div class=\"tip\" id=\"tip\"></div></div>\n"
               "<div class=\"sb\">\n");

    // Stats: Movement
    double cmdRate = (totalUs > 0 && ncmd > 1) ? (double)(ncmd-1) / ((double)totalUs / 1e6) : 0;
    double obsRate = (totalUs > 0 && nobs > 1) ? (double)(nobs-1) / ((double)totalUs / 1e6) : 0;
    fprintf(f, "<div class=\"sg\"><h3>Movement</h3>\n");
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Commands sent</span><span class=\"v\">%u</span></div>\n", ncmd);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Cursor polls</span><span class=\"v\">%u</span></div>\n", nobs);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Duration</span><span class=\"v\">%.1f ms</span></div>\n", (double)totalUs/1000.0);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Cmd rate</span><span class=\"v\">%.0f Hz</span></div>\n", cmdRate);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Obs rate</span><span class=\"v\">%.0f Hz</span></div>\n", obsRate);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Cmd bbox</span><span class=\"v\">%d&times;%d</span></div>\n", cMaxX-cMinX, cMaxY-cMinY);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Obs bbox</span><span class=\"v\">%.0f&times;%.0f</span></div>\n", oMaxX-oMinX, oMaxY-oMinY);
    fprintf(f, "</div>\n");

    // Stats: Accuracy
    const char* dCls = (devAvg < 2) ? "g" : (devAvg < 5) ? "w" : "b";
    fprintf(f, "<div class=\"sg\"><h3>Command Fidelity</h3>\n");
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Avg cmd/obs gap</span><span class=\"v %s\">%.2f px</span></div>\n", dCls, devAvg);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Max cmd/obs gap</span><span class=\"v\">%.2f px</span></div>\n", devMax);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Cmd delta repeats</span><span class=\"v\">%.0f%%</span></div>\n", cmdRepPct);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Cmd interval CV</span><span class=\"v\">%.3f</span></div>\n", intCV);
    fprintf(f, "</div>\n");

    // Humanization Analysis
    const char* scoreBg = (hScore < 10) ? "#3a1515" : (hScore < 25) ? "#3a2a15" :
                          (hScore < 50) ? "#2a3a15" : "#153a1a";
    fprintf(f, "<div class=\"sg\">\n");
    fprintf(f, "  <h3>Humanization Analysis</h3>\n");
    fprintf(f, "  <div style=\"background:%s;border-radius:4px;padding:8px 10px;margin-bottom:8px\">\n", scoreBg);
    fprintf(f, "    <div style=\"display:flex;justify-content:space-between;align-items:baseline\">\n");
    fprintf(f, "      <span style=\"font-size:20px;font-weight:bold\" class=\"v %s\">%d / 100</span>\n", hCls, (int)hScore);
    fprintf(f, "      <span style=\"font-size:13px\" class=\"v %s\">%s</span>\n", hCls, hGrade);
    fprintf(f, "    </div>\n");
    fprintf(f, "  </div>\n");

    // Individual factors with score contribution bars
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Perp scatter</span><span class=\"v\">%.3f px</span></div>\n", perpScatter);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Obs jitter (avg)</span><span class=\"v\">%.3f px</span></div>\n", jitAvg);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Obs jitter (max)</span><span class=\"v\">%.3f px</span></div>\n", jitMax);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Direction reversals</span><span class=\"v\">%.1f%% of samples</span></div>\n", dirFlipRate);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Speed variance (CV)</span><span class=\"v\">%.3f</span></div>\n", speedCV);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Sub-pixel noise</span><span class=\"v\">%.1f%%</span></div>\n", subPxPct);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Accel jerk</span><span class=\"v\">%.4f px/ms&sup2;</span></div>\n", accelJerk);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Path efficiency</span><span class=\"v\">%.4f</span></div>\n", pathEff);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Obs total dist</span><span class=\"v\">%.0f px</span></div>\n", obsTotalDist);
    fprintf(f, "  <div class=\"sr\"><span class=\"l\">Obs moving samples</span><span class=\"v\">%u / %u</span></div>\n", obsMoving, nobs > 0 ? nobs - 1 : 0);

    // Score breakdown legend
    fprintf(f, "  <div style=\"margin-top:8px;padding-top:6px;border-top:1px solid #1a1a24;font-size:10px;color:#444;line-height:1.6\">\n");
    fprintf(f, "    Perp scatter (25) + Obs jitter (20) + Dir reversals (15)<br>\n");
    fprintf(f, "    Speed CV (15) + Sub-px noise (10) + Accel jerk (10) + Path eff (5)\n");
    fprintf(f, "  </div>\n");
    fprintf(f, "</div>\n");

    // Interval histogram
    const char* bLabels[] = {"<1ms","1-2","2-5","5-10","10-20","20-50","50-100","100+"};
    uint32_t hMax = 1;
    for (int b = 0; b < 8; b++) if (cmdHist[b] > hMax) hMax = cmdHist[b];
    fprintf(f, "<div class=\"sg\"><h3>Command Intervals</h3>\n");
    for (int b = 0; b < 8; b++) {
        if (!cmdHist[b]) continue;
        fprintf(f, "  <div class=\"hb\"><span class=\"hl\">%s</span>"
                   "<div class=\"bar\" style=\"width:%.1f%%\"></div>"
                   "<span class=\"hc\">%u</span></div>\n",
                bLabels[b], (double)cmdHist[b]/hMax*100.0, cmdHist[b]);
    }
    fprintf(f, "</div>\n");

    // Controls
    fprintf(f,
        "<div class=\"ctrl\">\n"
        "  <h3 style=\"font-size:10px;text-transform:uppercase;letter-spacing:1.2px;color:#444;margin-bottom:6px;\">Display</h3>\n"
        "  <label><input type=\"checkbox\" id=\"chkCmd\" checked> Commanded path (blue)</label>\n"
        "  <label><input type=\"checkbox\" id=\"chkObs\" checked> Observed path (orange)</label>\n"
        "  <label><input type=\"checkbox\" id=\"chkDots\" checked> Sample dots</label>\n"
        "  <label><input type=\"checkbox\" id=\"chkDev\"> Deviation lines</label>\n"
        "  <label><input type=\"checkbox\" id=\"chkGrid\" checked> Grid</label>\n"
        "  <label><input type=\"checkbox\" id=\"chkSpeed\" checked> Color by speed</label>\n"
        "  <label>Line width <input type=\"range\" id=\"rngLW\" min=\"0.5\" max=\"4\" step=\"0.5\" value=\"1.5\"></label>\n"
        "  <label>Dot size <input type=\"range\" id=\"rngDS\" min=\"0.5\" max=\"3\" step=\"0.5\" value=\"1\"></label>\n"
        "  <div class=\"leg\">\n"
        "    <div class=\"leg-i\"><div class=\"leg-s\" style=\"background:#4ea8f0\"></div>Commanded</div>\n"
        "    <div class=\"leg-i\"><div class=\"leg-s\" style=\"background:#f07040\"></div>Observed</div>\n"
        "    <div class=\"leg-i\"><div class=\"leg-s\" style=\"background:#f04080;height:1px\"></div>Deviation</div>\n"
        "    <div class=\"leg-i\"><div class=\"leg-s\" style=\"background:#4ec970;border-radius:50%%;width:6px;height:6px\"></div>Start</div>\n"
        "    <div class=\"leg-i\"><div class=\"leg-s\" style=\"background:#e84848;border-radius:50%%;width:6px;height:6px\"></div>End</div>\n"
        "  </div>\n"
        "  <p style=\"margin-top:10px;color:#444;font-size:10px;\">Pan: drag &middot; Zoom: scroll &middot; Fit: double-click &middot; Hover: inspect</p>\n"
        "</div>\n");

    fprintf(f, "</div></div>\n");

    // ---- Emit data as JS typed arrays ----
    fprintf(f, "<script>\n");

    // Command channel
    fprintf(f, "const CT=new Float64Array([");
    for (uint32_t i = 0; i < ncmd; i++) {
        if (i) fputc(',', f);
        if (i % 20 == 0) fputc('\n', f);
        fprintf(f, "%.3f", (double)g_trace.cmds[i].time_us / 1000.0);
    }
    fprintf(f, "]);\nconst CX=new Int32Array([");
    for (uint32_t i = 0; i < ncmd; i++) {
        if (i) fputc(',', f);
        if (i % 30 == 0) fputc('\n', f);
        fprintf(f, "%d", g_trace.cmds[i].cum_x);
    }
    fprintf(f, "]);\nconst CY=new Int32Array([");
    for (uint32_t i = 0; i < ncmd; i++) {
        if (i) fputc(',', f);
        if (i % 30 == 0) fputc('\n', f);
        fprintf(f, "%d", g_trace.cmds[i].cum_y);
    }
    fprintf(f, "]);\nconst CDX=new Int16Array([");
    for (uint32_t i = 0; i < ncmd; i++) {
        if (i) fputc(',', f);
        if (i % 40 == 0) fputc('\n', f);
        fprintf(f, "%d", g_trace.cmds[i].dx);
    }
    fprintf(f, "]);\nconst CDY=new Int16Array([");
    for (uint32_t i = 0; i < ncmd; i++) {
        if (i) fputc(',', f);
        if (i % 40 == 0) fputc('\n', f);
        fprintf(f, "%d", g_trace.cmds[i].dy);
    }
    fprintf(f, "]);\n");

    // Observation channel
    fprintf(f, "const OT=new Float64Array([");
    for (uint32_t i = 0; i < nobs; i++) {
        if (i) fputc(',', f);
        if (i % 20 == 0) fputc('\n', f);
        fprintf(f, "%.3f", (double)g_trace.obs[i].time_us / 1000.0);
    }
    fprintf(f, "]);\nconst OX=new Float64Array([");
    for (uint32_t i = 0; i < nobs; i++) {
        if (i) fputc(',', f);
        if (i % 20 == 0) fputc('\n', f);
        fprintf(f, "%.2f", g_trace.obs[i].rel_x);
    }
    fprintf(f, "]);\nconst OY=new Float64Array([");
    for (uint32_t i = 0; i < nobs; i++) {
        if (i) fputc(',', f);
        if (i % 20 == 0) fputc('\n', f);
        fprintf(f, "%.2f", g_trace.obs[i].rel_y);
    }
    fprintf(f, "]);\n\n");

    // Rendering engine
    fprintf(f,
        "const cvs=document.getElementById('c'),ctx=cvs.getContext('2d'),tip=document.getElementById('tip');\n"
        "let dpr=devicePixelRatio||1,W,H;\n"
        "let vx=0,vy=0,vs=1;\n"
        "let drag=false,dsx,dsy,dvx,dvy;\n"
        "\n"
        "function resize(){\n"
        "  const r=cvs.parentElement.getBoundingClientRect();\n"
        "  W=r.width;H=r.height;\n"
        "  cvs.width=W*dpr;cvs.height=H*dpr;\n"
        "  cvs.style.width=W+'px';cvs.style.height=H+'px';\n"
        "  ctx.setTransform(dpr,0,0,dpr,0,0);\n"
        "}\n"
        "function fitView(){\n"
        "  const p=40,rx=%.2f,ry=%.2f;\n"
        "  vs=Math.min((W-p*2)/(rx||10),(H-p*2)/(ry||10))*0.9;\n"
        "  vx=W/2-(%.2f)*vs; vy=H/2-(%.2f)*vs;\n"
        "}\n"
        "function toS(px,py){return[px*vs+vx,py*vs+vy];}\n"
        "function toD(sx,sy){return[(sx-vx)/vs,(sy-vy)/vs];}\n"
        "\n",
        rangeX, rangeY,
        allMinX + rangeX / 2, allMinY + rangeY / 2);

    fprintf(f,
        "function spdColor(s,mx){\n"
        "  const t=Math.min(s/(mx*0.5+.001),1);\n"
        "  if(t<.33)return`rgb(${70+t*3*100|0},${140+t*3*60|0},240)`;\n"
        "  if(t<.66){const u=(t-.33)*3;return`rgb(${70+u*180|0},${200-u*20|0},${240-u*200|0})`;}\n"
        "  const u=(t-.66)*3;return`rgb(240,${180-u*140|0},${40-u*20|0})`;\n"
        "}\n"
        "\n"
        "function calcSpd(T,X,Y){\n"
        "  const n=T.length,s=new Float64Array(n);\n"
        "  let mx=0;\n"
        "  for(let i=1;i<n;i++){\n"
        "    const dt=T[i]-T[i-1];\n"
        "    if(dt<=0){s[i]=s[i-1];continue;}\n"
        "    const dx=X[i]-X[i-1],dy=Y[i]-Y[i-1];\n"
        "    s[i]=Math.sqrt(dx*dx+dy*dy)/dt;\n"
        "    if(s[i]>mx)mx=s[i];\n"
        "  }\n"
        "  return{s,mx};\n"
        "}\n"
        "const cSpd=calcSpd(CT,CX,CY),oSpd=calcSpd(OT,OX,OY);\n"
        "const gMx=Math.max(cSpd.mx,oSpd.mx);\n\n");

    fprintf(f,
        "function draw(){\n"
        "  ctx.clearRect(0,0,W,H);\n"
        "  const sCmd=document.getElementById('chkCmd').checked;\n"
        "  const sObs=document.getElementById('chkObs').checked;\n"
        "  const sDots=document.getElementById('chkDots').checked;\n"
        "  const sDev=document.getElementById('chkDev').checked;\n"
        "  const sGrid=document.getElementById('chkGrid').checked;\n"
        "  const sSpd=document.getElementById('chkSpeed').checked;\n"
        "  const lw=+document.getElementById('rngLW').value;\n"
        "  const ds=+document.getElementById('rngDS').value;\n"
        "\n"
        "  // Grid\n"
        "  if(sGrid){\n"
        "    const gs=Math.pow(10,Math.floor(Math.log10(100/vs)));\n"
        "    const[dMx,dMy]=toD(0,0),[dXx,dXy]=toD(W,H);\n"
        "    ctx.strokeStyle='#161620';ctx.lineWidth=.5;\n"
        "    ctx.font='9px monospace';ctx.fillStyle='#2a2a36';\n"
        "    for(let gx=Math.floor(dMx/gs)*gs;gx<=dXx;gx+=gs){\n"
        "      const sx=gx*vs+vx;\n"
        "      ctx.beginPath();ctx.moveTo(sx,0);ctx.lineTo(sx,H);ctx.stroke();\n"
        "      if(Math.abs(gx)<.01){ctx.save();ctx.strokeStyle='#252535';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(sx,0);ctx.lineTo(sx,H);ctx.stroke();ctx.restore();}\n"
        "      ctx.fillText(gx.toFixed(0),sx+2,12);\n"
        "    }\n"
        "    for(let gy=Math.floor(dMy/gs)*gs;gy<=dXy;gy+=gs){\n"
        "      const sy=gy*vs+vy;\n"
        "      ctx.beginPath();ctx.moveTo(0,sy);ctx.lineTo(W,sy);ctx.stroke();\n"
        "      if(Math.abs(gy)<.01){ctx.save();ctx.strokeStyle='#252535';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,sy);ctx.lineTo(W,sy);ctx.stroke();ctx.restore();}\n"
        "      ctx.fillText(gy.toFixed(0),2,sy-2);\n"
        "    }\n"
        "  }\n"
        "\n"
        "  // Origin crosshair\n"
        "  const[ox,oy]=toS(0,0);\n"
        "  ctx.strokeStyle='#333340';ctx.lineWidth=1;\n"
        "  ctx.beginPath();ctx.arc(ox,oy,6,0,Math.PI*2);ctx.stroke();\n"
        "  ctx.beginPath();ctx.moveTo(ox-8,oy);ctx.lineTo(ox+8,oy);ctx.stroke();\n"
        "  ctx.beginPath();ctx.moveTo(ox,oy-8);ctx.lineTo(ox,oy+8);ctx.stroke();\n"
        "\n");

    // Deviation lines
    fprintf(f,
        "  // Deviation lines between cmd and obs at matching timestamps\n"
        "  if(sDev&&sCmd&&sObs){\n"
        "    ctx.strokeStyle='rgba(240,64,128,0.3)';ctx.lineWidth=.5;\n"
        "    let oi=0;\n"
        "    const step=Math.max(1,Math.floor(CT.length/300));\n"
        "    if(OT.length>=2){\n"
        "    for(let ci=0;ci<CT.length;ci+=step){\n"
        "      if(CT[ci]<OT[0])continue;\n"
        "      if(CT[ci]>OT[OT.length-1])break;\n"
        "      while(oi+1<OT.length&&OT[oi+1]<=CT[ci])oi++;\n"
        "      if(oi+1>=OT.length)break;\n"
        "      const t0=OT[oi],t1=OT[oi+1];\n"
        "      if(t1<=t0||CT[ci]<t0)continue;\n"
        "      const fr=(CT[ci]-t0)/(t1-t0);\n"
        "      if(fr<0||fr>1)continue;\n"
        "      const ix=OX[oi]+fr*(OX[oi+1]-OX[oi]);\n"
        "      const iy=OY[oi]+fr*(OY[oi+1]-OY[oi]);\n"
        "      const[x1,y1]=toS(CX[ci],CY[ci]),[x2,y2]=toS(ix,iy);\n"
        "      ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();\n"
        "    }}\n"
        "  }\n\n");

    // Path drawing function
    fprintf(f,
        "  function drawPath(T,X,Y,spd,color,mx){\n"
        "    const n=T.length;if(n<2)return;\n"
        "    if(sSpd){\n"
        "      for(let i=1;i<n;i++){\n"
        "        const[x0,y0]=toS(X[i-1],Y[i-1]),[x1,y1]=toS(X[i],Y[i]);\n"
        "        ctx.strokeStyle=spdColor(spd.s[i],mx);ctx.lineWidth=lw;\n"
        "        ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x1,y1);ctx.stroke();\n"
        "      }\n"
        "    } else {\n"
        "      ctx.strokeStyle=color;ctx.lineWidth=lw;ctx.beginPath();\n"
        "      const[sx,sy]=toS(X[0],Y[0]);ctx.moveTo(sx,sy);\n"
        "      for(let i=1;i<n;i++){const[px,py]=toS(X[i],Y[i]);ctx.lineTo(px,py);}\n"
        "      ctx.stroke();\n"
        "    }\n"
        "    if(sDots){\n"
        "      const step=Math.max(1,Math.floor(n/3000));\n"
        "      for(let i=0;i<n;i+=step){\n"
        "        const[px,py]=toS(X[i],Y[i]);\n"
        "        ctx.fillStyle=sSpd?spdColor(spd.s[i],mx):color;\n"
        "        ctx.beginPath();ctx.arc(px,py,ds,0,Math.PI*2);ctx.fill();\n"
        "      }\n"
        "    }\n"
        "    // Start (green) and end (red) markers\n"
        "    const[sx,sy]=toS(X[0],Y[0]),[ex,ey]=toS(X[n-1],Y[n-1]);\n"
        "    ctx.fillStyle='#4ec970';ctx.beginPath();ctx.arc(sx,sy,4,0,Math.PI*2);ctx.fill();\n"
        "    ctx.fillStyle='#e84848';ctx.beginPath();ctx.arc(ex,ey,4,0,Math.PI*2);ctx.fill();\n"
        "  }\n"
        "\n"
        "  // Observed underneath, commanded on top\n"
        "  if(sObs)drawPath(OT,OX,OY,oSpd,'#f07040',gMx);\n"
        "  if(sCmd)drawPath(CT,CX,CY,cSpd,'#4ea8f0',gMx);\n"
        "}\n\n");

    // Interaction
    fprintf(f,
        "cvs.onmousedown=e=>{if(!e.button){drag=true;dsx=e.clientX;dsy=e.clientY;dvx=vx;dvy=vy;}};\n"
        "onmousemove=e=>{\n"
        "  if(drag){vx=dvx+(e.clientX-dsx);vy=dvy+(e.clientY-dsy);draw();}\n"
        "  const r=cvs.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;\n"
        "  let best=-1,bestD=1e9;\n"
        "  for(let i=0;i<CT.length;i++){\n"
        "    const[sx,sy]=toS(CX[i],CY[i]);\n"
        "    const d=Math.hypot(sx-mx,sy-my);\n"
        "    if(d<bestD){bestD=d;best=i;}\n"
        "  }\n"
        "  if(best>=0&&bestD<15){\n"
        "    tip.style.display='block';\n"
        "    tip.style.left=(e.clientX-r.left+14)+'px';\n"
        "    tip.style.top=(e.clientY-r.top-10)+'px';\n"
        "    tip.innerHTML='<b>Cmd #'+best+'</b><br>'+\n"
        "      't='+CT[best].toFixed(1)+'ms<br>'+\n"
        "      'pos=('+CX[best]+', '+CY[best]+')<br>'+\n"
        "      'delta=('+CDX[best]+', '+CDY[best]+')<br>'+\n"
        "      'speed='+cSpd.s[best].toFixed(2)+' px/ms';\n"
        "  }else{tip.style.display='none';}\n"
        "};\n"
        "onmouseup=()=>{drag=false;};\n"
        "cvs.onwheel=e=>{\n"
        "  e.preventDefault();\n"
        "  const r=cvs.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;\n"
        "  const f=e.deltaY<0?1.15:1/1.15;\n"
        "  vx=mx-(mx-vx)*f;vy=my-(my-vy)*f;vs*=f;draw();\n"
        "};\n"
        "cvs.ondblclick=()=>{fitView();draw();};\n"
        "document.querySelectorAll('.ctrl input').forEach(el=>el.oninput=draw);\n"
        "resize();fitView();draw();\n"
        "onresize=()=>{resize();fitView();draw();};\n");

    fprintf(f, "</script>\n</body>\n</html>\n");
    fclose(f);

    printf("  [TRACE] Wrote: %s\n", filename);

    // Auto-open in default browser on macOS
    char open_cmd[600];
    snprintf(open_cmd, sizeof(open_cmd), "open \"%s\"", filename);
    system(open_cmd);
}

// ============================================================================
// Serial Reader Thread
// ============================================================================

void* serial_reader_thread_fn(void* arg) {
    (void)arg;
    char buffer[256];
    int buf_pos = 0;
    uint32_t byte_count = 0;

    while (g_config.running) {
        char c;
        ssize_t n = read(g_serial_fd, &c, 1);
        if (n > 0) {
            byte_count++;
            if (g_config.verbose && byte_count < 100)
                printf("[RX BYTE %u] 0x%02X '%c'\n", byte_count, (unsigned char)c, (c>=32&&c<127)?c:'.');
            if (c == '\n' || c == '\r') {
                if (buf_pos > 0) {
                    buffer[buf_pos] = '\0';
                    if (strcmp(buffer, "ok") == 0 || strcmp(buffer, ">>>") == 0) {
                        __sync_fetch_and_add(&g_stats.responses_ok, 1);
                        g_stats.last_response_time = time(NULL);
                    } else if (strncmp(buffer, "ERR", 3) == 0) {
                        __sync_fetch_and_add(&g_stats.responses_err, 1);
                    }
                    if (g_config.verbose) printf("[RX] '%s'\n", buffer);
                    buf_pos = 0;
                }
            } else if (buf_pos < (int)sizeof(buffer) - 1) {
                buffer[buf_pos++] = c;
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            break;
        } else {
            usleep(1000);
        }
    }
    return NULL;
}

// ============================================================================
// Serial Sender Thread
// ============================================================================

void* serial_sender_thread_fn(void* arg) {
    (void)arg;
    uint32_t send_count = 0, drop_count = 0;
    printf("[SENDER] Thread started\n");

    while (g_config.running) {
        uint32_t head = g_move_queue_head;
        uint32_t tail = g_move_queue_tail;
        if (head != tail) {
            int32_t total_dx = 0, total_dy = 0;
            int coalesced = 0;
            while (head != tail && coalesced < 32) {
                move_cmd_t cmd = g_move_queue[tail & MOVE_QUEUE_MASK];
                total_dx += cmd.dx; total_dy += cmd.dy;
                tail = (tail + 1) & MOVE_QUEUE_MASK;
                coalesced++;
            }
            __sync_synchronize();
            g_move_queue_tail = tail;

            if (total_dx > 127) total_dx = 127;
            if (total_dx < -127) total_dx = -127;
            if (total_dy > 127) total_dy = 127;
            if (total_dy < -127) total_dy = -127;

            char buf[32];
            int len = snprintf(buf, sizeof(buf), "km.move(%d, %d)\n", (int)total_dx, (int)total_dy);
            int offset = 0, retries = 0;
            while (offset < len && retries < 100) {
                ssize_t written = write(g_serial_fd, buf + offset, len - offset);
                if (written > 0) { offset += written; retries = 0; }
                else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { usleep(2000); retries++; }
                else { __sync_fetch_and_add(&g_stats.errors, 1); break; }
            }
            if (offset == len) {
                send_count++;
                __sync_fetch_and_add(&g_stats.packets_sent, 1);
                __sync_fetch_and_add(&g_stats.movements_countered, coalesced);
            } else if (retries >= 100) { drop_count++; }
            usleep(4000);
        } else {
            usleep(500);
        }
    }
    printf("[SENDER] Exiting. Sent: %u, dropped: %u\n", send_count, drop_count);
    return NULL;
}

// ============================================================================
// Serial Functions
// ============================================================================

int serial_open(const char* port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { fprintf(stderr, "Error: Cannot open %s: %s\n", port, strerror(errno)); return -1; }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed); cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~(PARENB|CSTOPB|CSIZE|CRTSCTS)) | CS8 | CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ISIG);
    tty.c_iflag &= ~(IXON|IXOFF|IXANY|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~(OPOST|ONLCR);
    tty.c_cc[VTIME] = 1; tty.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

bool send_mouse_move(int fd, int16_t x, int16_t y) {
    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "km.move(%d, %d)\n", x, y);
    pthread_mutex_lock(&g_serial_mutex);
    ssize_t written = write(fd, cmd, len);
    int we = errno;
    pthread_mutex_unlock(&g_serial_mutex);
    if (written == len) { __sync_fetch_and_add(&g_stats.packets_sent, 1); return true; }
    if (written < 0 && (we == EAGAIN || we == EWOULDBLOCK)) return false;
    __sync_fetch_and_add(&g_stats.errors, 1);
    return false;
}

bool send_transform(int fd, int16_t sx, int16_t sy, bool en) {
    char cmd[64];
    int len = snprintf(cmd, sizeof(cmd), "km.transform(%d, %d, %d)\n", sx, sy, en?1:0);
    pthread_mutex_lock(&g_serial_mutex);
    ssize_t written = write(fd, cmd, len);
    pthread_mutex_unlock(&g_serial_mutex);
    return written == len;
}

bool send_direct_move(int16_t dx, int16_t dy) {
    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "km.move(%d, %d)\n", dx, dy);
    pthread_mutex_lock(&g_serial_mutex);
    ssize_t written = write(g_serial_fd, cmd, len);
    pthread_mutex_unlock(&g_serial_mutex);
    if (written == len) { __sync_fetch_and_add(&g_stats.packets_sent, 1); return true; }
    return false;
}

// ============================================================================
// CGEventTap Callback
// ============================================================================

CGEventRef mouse_event_callback(CGEventTapProxy proxy, CGEventType type,
                                 CGEventRef event, void* userInfo) {
    (void)proxy; (void)userInfo;
    if (type == kCGEventTapDisabledByTimeout || type == kCGEventTapDisabledByUserInput) {
        CGEventTapEnable(g_event_tap, true);
        return event;
    }
    if (type != kCGEventMouseMoved && type != kCGEventLeftMouseDragged &&
        type != kCGEventRightMouseDragged && type != kCGEventOtherMouseDragged)
        return event;

    int64_t dx64 = CGEventGetIntegerValueField(event, kCGMouseEventDeltaX);
    int64_t dy64 = CGEventGetIntegerValueField(event, kCGMouseEventDeltaY);
    if (dx64 == 0 && dy64 == 0) return event;

    int16_t dx = (int16_t)dx64, dy = (int16_t)dy64;
    __sync_fetch_and_add(&g_stats.movements_detected, 1);
    __sync_fetch_and_add(&g_stats.total_dx, labs(dx));
    __sync_fetch_and_add(&g_stats.total_dy, labs(dy));

    if (!g_config.active) return event;
    if (abs(dx) < g_config.deadzone && abs(dy) < g_config.deadzone) return event;

    int16_t cdx = (int16_t)(-dx * g_config.gain), cdy = (int16_t)(-dy * g_config.gain);
    uint32_t head = g_move_queue_head;
    uint32_t next = (head + 1) & MOVE_QUEUE_MASK;
    if (next != g_move_queue_tail) {
        g_move_queue[head & MOVE_QUEUE_MASK].dx = cdx;
        g_move_queue[head & MOVE_QUEUE_MASK].dy = cdy;
        __sync_synchronize();
        g_move_queue_head = next;
    }
    return event;
}

// ============================================================================
// Event Tap Setup
// ============================================================================

bool setup_event_tap(void) {
    if (!AXIsProcessTrusted()) {
        fprintf(stderr, "\n  Accessibility permission required.\n"
                        "  System Preferences > Security & Privacy > Privacy > Accessibility\n\n");
        const void* keys[] = { kAXTrustedCheckOptionPrompt };
        const void* values[] = { kCFBooleanTrue };
        CFDictionaryRef opts = CFDictionaryCreate(kCFAllocatorDefault, keys, values, 1,
            &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
        AXIsProcessTrustedWithOptions(opts);
        CFRelease(opts);
        return false;
    }
    CGEventMask mask = CGEventMaskBit(kCGEventMouseMoved) | CGEventMaskBit(kCGEventLeftMouseDragged) |
                       CGEventMaskBit(kCGEventRightMouseDragged) | CGEventMaskBit(kCGEventOtherMouseDragged);
    g_event_tap = CGEventTapCreate(kCGSessionEventTap, kCGHeadInsertEventTap,
                                    kCGEventTapOptionListenOnly, mask, mouse_event_callback, NULL);
    if (!g_event_tap) { fprintf(stderr, "Error: Failed to create event tap\n"); return false; }
    g_run_loop_source = CFMachPortCreateRunLoopSource(kCFAllocatorDefault, g_event_tap, 0);
    if (!g_run_loop_source) { CFRelease(g_event_tap); g_event_tap = NULL; return false; }
    CFRunLoopAddSource(CFRunLoopGetCurrent(), g_run_loop_source, kCFRunLoopCommonModes);
    CGEventTapEnable(g_event_tap, true);
    return true;
}

void cleanup_event_tap(void) {
    if (g_event_tap) CGEventTapEnable(g_event_tap, false);
    if (g_run_loop_source) {
        CFRunLoopRemoveSource(CFRunLoopGetCurrent(), g_run_loop_source, kCFRunLoopCommonModes);
        CFRelease(g_run_loop_source); g_run_loop_source = NULL;
    }
    if (g_event_tap) { CFRelease(g_event_tap); g_event_tap = NULL; }
}

// ============================================================================
// Statistics
// ============================================================================

void print_stats(void) {
    time_t elapsed = time(NULL) - g_stats.start_time;
    if (elapsed == 0) elapsed = 1;
    printf("\n=== Statistics ===\n");
    printf("Runtime:     %ld s\n", elapsed);
    printf("Detected:    %lld (%.1f/s)\n", (long long)g_stats.movements_detected,
           (double)g_stats.movements_detected / elapsed);
    printf("Countered:   %lld\n", (long long)g_stats.movements_countered);
    printf("Packets:     %lld\n", (long long)g_stats.packets_sent);
    printf("OK/ERR:      %lld / %lld\n", (long long)g_stats.responses_ok, (long long)g_stats.responses_err);
    printf("Errors:      %lld\n", (long long)g_stats.errors);
    printf("Gain:        %.2f\n", g_config.gain);
    printf("==================\n\n");
}

// ============================================================================
// Terminal & Keyboard
// ============================================================================

static struct termios g_old_term;
static bool g_term_saved = false;

void set_terminal_mode(bool raw) {
    if (raw) {
        struct termios nt;
        tcgetattr(STDIN_FILENO, &g_old_term); g_term_saved = true;
        nt = g_old_term;
        nt.c_lflag &= ~(ICANON|ECHO);
        nt.c_cc[VMIN] = 0; nt.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &nt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else if (g_term_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_old_term);
        fcntl(STDIN_FILENO, F_SETFL, 0);
    }
}

void process_keyboard_input(void) {
    char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return;
    if (c == 27) {
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) == 1 && seq[0] == '[' && read(STDIN_FILENO, &seq[1], 1) == 1) {
            switch (seq[1]) {
                case 'A': send_direct_move(0, -g_move_step); return;
                case 'B': send_direct_move(0, g_move_step); return;
                case 'C': send_direct_move(g_move_step, 0); return;
                case 'D': send_direct_move(-g_move_step, 0); return;
            }
        } else {
            g_config.running = false; CFRunLoopStop(CFRunLoopGetCurrent()); return;
        }
    }
    switch (c) {
        case 'w': case 'W': send_direct_move(0, -g_move_step); break;
        case 'a': case 'A': send_direct_move(-g_move_step, 0); break;
        case 's': case 'S': send_direct_move(0, g_move_step); break;
        case 'd': case 'D': send_direct_move(g_move_step, 0); break;
        case 'i': case 'I': send_direct_move(0, -g_move_step); break;
        case 'j': case 'J': send_direct_move(-g_move_step, 0); break;
        case 'k': case 'K': send_direct_move(0, g_move_step); break;
        case 'l': case 'L': send_direct_move(g_move_step, 0); break;
        case ' ':
            g_config.active = !g_config.active;
            printf("\n>>> Counteraction %s <<<\n\n", g_config.active ? "ENABLED" : "DISABLED");
            break;
        case 'p': case 'P': case '?': print_stats(); break;
        case 't': case 'T':
            g_transform_enabled = !g_transform_enabled;
            if (g_transform_enabled) { send_transform(g_serial_fd, 0, 0, true); printf("\n>>> Transform ON <<<\n\n"); }
            else { send_transform(g_serial_fd, 256, 256, false); printf("\n>>> Transform OFF <<<\n\n"); }
            break;
        case '+': case '=': g_config.gain = fminf(2.0f, g_config.gain + 0.1f); printf("\n>>> Gain: %.2f <<<\n\n", g_config.gain); break;
        case '-': case '_': g_config.gain = fmaxf(0.0f, g_config.gain - 0.1f); printf("\n>>> Gain: %.2f <<<\n\n", g_config.gain); break;
        case ']': g_move_step = (g_move_step < 200) ? g_move_step + 5 : 200; printf("\n>>> Step: %d <<<\n\n", g_move_step); break;
        case '[': g_move_step = (g_move_step > 5) ? g_move_step - 5 : 5; printf("\n>>> Step: %d <<<\n\n", g_move_step); break;
        case 'q': case 'Q': g_config.running = false; CFRunLoopStop(CFRunLoopGetCurrent()); break;
        case 'v': case 'V': g_config.verbose = !g_config.verbose; printf("\n>>> Verbose: %s <<<\n\n", g_config.verbose?"ON":"OFF"); break;
        default: break;
    }
}

void live_stats_timer_callback(CFRunLoopTimerRef timer, void* info) {
    (void)timer; (void)info;
    if (!g_show_live_stats || !g_config.running) return;
    time_t el = time(NULL) - g_stats.start_time; if (!el) el = 1;
    printf("\r[LIVE] Det:%lld Ctr:%lld OK:%lld ERR:%lld %.1f/s    ",
           (long long)g_stats.movements_detected, (long long)g_stats.movements_countered,
           (long long)g_stats.responses_ok, (long long)g_stats.responses_err,
           (double)g_stats.movements_detected / el);
    fflush(stdout);
}

void keyboard_timer_callback(CFRunLoopTimerRef timer, void* info) {
    (void)timer; (void)info;
    if (!g_config.running) { CFRunLoopStop(CFRunLoopGetCurrent()); return; }
    process_keyboard_input();
}

void signal_handler(int sig) {
    (void)sig;
    g_config.running = false;
    CFRunLoopStop(CFRunLoopGetMain());
}

// ============================================================================
// Argument Parsing
// ============================================================================

void print_usage(const char* prog) {
    printf("Mouse Movement Counteraction Tool (macOS)\n\n");
    printf("Usage: %s [OPTIONS] <serial_port>\n\n", prog);
    printf("Options:\n");
    printf("  -b, --baud RATE      Baud rate (default: 2000000)\n");
    printf("  -g, --gain FLOAT     Counteraction gain 0.0-2.0 (default: 1.0)\n");
    printf("  -d, --deadzone PX    Ignore small movements (default: 0)\n");
    printf("  -v, --verbose        Verbose logging\n");
    printf("  -p, --paused         Start paused\n");
    printf("  -o, --output DIR     Trace output directory (default: .)\n");
    printf("  -t, --test MODE      Test mode (see below)\n");
    printf("  -h, --help           Show help\n\n");
    printf("Test modes:\n");
    printf("  Basic:       rapid, precise, flicks, sweep, mixed\n");
    printf("  Aimbot sim:  aim_approach, aim_flick, aim_recoil, aim_track, aim_full\n");
    printf("  Diagnostics: diag_tremor, diag_line, diag_repeat, diag_overshoot, diag_ease\n");
    printf("  Combined:    aim_all, diag_all, all (everything)\n\n");
    printf("Each test writes a trace_<name>_<timestamp>.html with:\n");
    printf("  Blue path  = Commanded (what we sent to KMBox)\n");
    printf("  Orange path = Observed (actual cursor position polled from macOS)\n");
    printf("  Pink lines  = Deviation between commanded and observed\n\n");
}

bool parse_args(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { print_usage(argv[0]); return false; }
        else if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose")) g_config.verbose = true;
        else if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--paused")) g_config.active = false;
        else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--test")) {
            if (++i >= argc) { fprintf(stderr, "Missing test mode\n"); return false; }
            strncpy(g_config.test_mode, argv[i], sizeof(g_config.test_mode) - 1);
        }
        else if (!strcmp(argv[i], "-o") || !strcmp(argv[i], "--output")) {
            if (++i >= argc) { fprintf(stderr, "Missing output dir\n"); return false; }
            strncpy(g_config.output_dir, argv[i], sizeof(g_config.output_dir) - 1);
        }
        else if (!strcmp(argv[i], "-b") || !strcmp(argv[i], "--baud")) {
            if (++i >= argc) { fprintf(stderr, "Missing baud\n"); return false; }
            g_config.baudrate = atoi(argv[i]);
        }
        else if (!strcmp(argv[i], "-g") || !strcmp(argv[i], "--gain")) {
            if (++i >= argc) { fprintf(stderr, "Missing gain\n"); return false; }
            g_config.gain = atof(argv[i]);
            if (g_config.gain < 0 || g_config.gain > 2) { fprintf(stderr, "Gain 0-2\n"); return false; }
        }
        else if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--deadzone")) {
            if (++i >= argc) { fprintf(stderr, "Missing deadzone\n"); return false; }
            g_config.deadzone = atoi(argv[i]);
        }
        else if (argv[i][0] != '-') strncpy(g_config.port, argv[i], sizeof(g_config.port) - 1);
    }
    if (!g_config.port[0]) { fprintf(stderr, "No serial port specified\n\n"); print_usage(argv[0]); return false; }
    return true;
}

// ============================================================================
// Test Functions - with Trace Recording
// ============================================================================

void send_move_traced(int dx, int dy) {
    trace_record_cmd((int16_t)dx, (int16_t)dy);
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "km.move(%d, %d)\n", dx, dy);
    ssize_t w = write(g_serial_fd, buf, len);
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // Buffer full - brief drain wait then retry once
        usleep(200);
        write(g_serial_fd, buf, len);
    }
    // Minimal pacing: ~20 byte cmd at USB CDC speeds needs ~10μs,
    // but give the bridge firmware time to process
    usleep(100);
}

void test_rapid_small_movements(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Rapid Small Movements (10k iterations ±10px)         ║\n");
    printf("║  Max throughput - no pacing delays                          ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("rapid");
    time_t start = time(NULL);
    for (int i = 0; i < 10000; i++) {
        send_move_traced(0, 10);
        send_move_traced(0, -10);
        if (i % 2000 == 0) printf("  %d/10000\n", i);
    }
    time_t el = time(NULL) - start; if (!el) el = 1;
    trace_stop();
    printf("  ✅ %.0f cmd/sec\n", 20000.0 / el);
    trace_write_html("rapid");
}

void test_small_precise_movements(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Small Precise Movements (1-5px @ 500Hz)              ║\n");
    printf("║  Humanization should add visible perpendicular scatter      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    int mv[][2] = {{1,0},{0,1},{-1,0},{0,-1},{2,0},{0,2},{-2,0},{0,-2},
                   {3,3},{-3,3},{-3,-3},{3,-3},{5,0},{0,5},{-5,0},{0,-5}};
    int nm = sizeof(mv)/sizeof(mv[0]);
    trace_start("precise");
    usleep(200000);  // 200ms settle
    for (int r = 0; r < 40; r++)
        for (int i = 0; i < nm; i++) { send_move_traced(mv[i][0], mv[i][1]); usleep(2000); }
    trace_stop();
    printf("  ✅ Complete (%d commands)\n", 40 * nm);
    trace_write_html("precise");
}

void test_large_fast_flicks(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Large Fast Flicks (±127px @ 125Hz)                   ║\n");
    printf("║  Should be snappy with minimal perpendicular deviation      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    int fl[][2] = {{127,0},{-127,0},{0,127},{0,-127},{90,90},{-90,-90},{90,-90},{-90,90}};
    int nf = sizeof(fl)/sizeof(fl[0]);
    trace_start("flicks");
    usleep(200000);
    for (int r = 0; r < 20; r++)
        for (int i = 0; i < nf; i++) { send_move_traced(fl[i][0], fl[i][1]); usleep(8000); }
    trace_stop();
    printf("  ✅ Complete (%d commands)\n", 20 * nf);
    trace_write_html("flicks");
}

void test_horizontal_sweep(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Horizontal Sweep (1kHz smooth tracking)              ║\n");
    printf("║  Ideal for seeing perpendicular jitter on a straight line   ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("sweep");
    printf("  Sweeping right (500 steps)...\n");
    for (int i = 0; i < 500; i++) { send_move_traced(5, 0); usleep(1000); }
    usleep(100000);  // 100ms pause at turnaround
    printf("  Sweeping left (500 steps)...\n");
    for (int i = 0; i < 500; i++) { send_move_traced(-5, 0); usleep(1000); }
    trace_stop();
    printf("  ✅ Complete (1000 commands)\n");
    trace_write_html("sweep");
}

void test_mixed_pattern(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Mixed Real-World Pattern (aim simulation @ 500Hz)    ║\n");
    printf("║  Micro-adjustments + flicks, like actual recoil control     ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("mixed");
    for (int c = 0; c < 8; c++) {
        printf("  Cycle %d/8\n", c + 1);
        // Slow tracking (small adjustments at 500Hz)
        for (int i = 0; i < 100; i++) { send_move_traced(2, 1); usleep(2000); }
        // Quick flick
        send_move_traced(-100, -50); usleep(20000);
        // Recovery micro-adjustments
        for (int i = 0; i < 60; i++) { send_move_traced(-1, 2); usleep(2000); }
        // Snap back
        send_move_traced(80, 40); usleep(30000);
        // Rapid burst (recoil compensation style)
        for (int i = 0; i < 40; i++) { send_move_traced(0, -3); usleep(1000); }
    }
    trace_stop();
    printf("  ✅ Complete\n");
    trace_write_html("mixed");
}

// ============================================================================
// Aimbot-Pattern Tests (replicates eden13378 AimBot.hpp movement signatures)
//
// The original code does this each frame (~60Hz):
//   TargetX = (ScreenPos.x - ScreenCenterX) / FakeSmooth   (FakeSmooth=0.8)
//   TargetY = (ScreenPos.y - ScreenCenterY) / FakeSmooth
//   Move_Auto(TargetX, TargetY, 60 * Smooth)               (Smooth=0.3 → 18ms)
//
// This is a proportional controller: each frame sends (remaining / 0.8) as
// a move interpolated over 18ms. Since 1/0.8 = 1.25, it overshoots, creating
// an oscillating convergence pattern that's highly detectable.
//
// We simulate the frame-by-frame deltas this produces and send them to KMBox
// to test whether firmware humanization makes these patterns look natural.
// ============================================================================

// Simulate the proportional controller convergence toward a target.
// fake_smooth: divisor (0.8 = overshoot, 1.5 = slow approach)
// frame_us: microseconds per frame (16667 = 60Hz, 8333 = 120Hz)
// Returns when within 0.5px of target or max_frames reached.
static void aimbot_converge(float target_x, float target_y,
                            float fake_smooth, int frame_us, int max_frames)
{
    float remaining_x = target_x;
    float remaining_y = target_y;

    for (int f = 0; f < max_frames; f++) {
        // This is what the aimbot computes each frame
        float move_x = remaining_x / fake_smooth;
        float move_y = remaining_y / fake_smooth;

        // Clamp to int8 range (KMBox HID report limits)
        int dx = (int)move_x;
        int dy = (int)move_y;
        if (dx > 127) dx = 127; if (dx < -127) dx = -127;
        if (dy > 127) dy = 127; if (dy < -127) dy = -127;

        // Skip zero moves (target reached)
        if (dx == 0 && dy == 0) break;

        send_move_traced(dx, dy);

        // Simulate: cursor moved by dx,dy, so remaining decreases
        remaining_x -= dx;
        remaining_y -= dy;

        // Check convergence
        if (fabsf(remaining_x) < 0.5f && fabsf(remaining_y) < 0.5f) break;

        usleep(frame_us);
    }
}

void test_aimbot_approach(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Aimbot Approach (proportional controller @ 60Hz)     ║\n");
    printf("║  Simulates target lock-on from various distances            ║\n");
    printf("║  Pattern: exponential decay with overshoot (÷0.8 each frame)║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("aim_approach");
    usleep(200000);

    // Close target: 50px right, 30px down (pistol micro-adjust)
    printf("  Close target (50, 30)...\n");
    aimbot_converge(50, 30, 0.8f, 16667, 60);
    usleep(500000);
    // Return to origin
    aimbot_converge(-50, -30, 0.8f, 16667, 60);
    usleep(500000);

    // Medium target: 200px right, 100px up (rifle spray transfer)
    printf("  Medium target (200, -100)...\n");
    aimbot_converge(200, -100, 0.8f, 16667, 120);
    usleep(500000);
    aimbot_converge(-200, 100, 0.8f, 16667, 120);
    usleep(500000);

    // Far target: 500px diagonal (AWP flick)
    printf("  Far target (500, -300)...\n");
    aimbot_converge(500, -300, 0.8f, 16667, 200);
    usleep(500000);
    aimbot_converge(-500, 300, 0.8f, 16667, 200);
    usleep(300000);

    // High smooth (1.5) - slow approach for comparison
    printf("  Slow approach smooth=1.5 (200, 100)...\n");
    aimbot_converge(200, 100, 1.5f, 16667, 200);
    usleep(500000);
    aimbot_converge(-200, -100, 1.5f, 16667, 200);

    trace_stop();
    printf("  ✅ Complete\n");
    trace_write_html("aim_approach");
}

void test_aimbot_flick(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Aimbot Flick Sequences (target switching)            ║\n");
    printf("║  Snap to target → exponential settle → snap to next         ║\n");
    printf("║  This is the most detectable pattern: instant direction     ║\n");
    printf("║  change followed by perfectly geometric deceleration        ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("aim_flick");
    usleep(200000);

    // Flick sequence: snap between 5 targets
    struct { float x, y; } targets[] = {
        { 300, -50},   // Target 1: far right, slightly up
        {-150,  200},  // Target 2: left and down
        { 400,  100},  // Target 3: far right, down
        {-250, -150},  // Target 4: left and up
        { 100,   50},  // Target 5: close right, down
    };
    int ntargets = sizeof(targets) / sizeof(targets[0]);

    for (int rep = 0; rep < 3; rep++) {
        printf("  Flick sequence %d/3\n", rep + 1);
        float cur_x = 0, cur_y = 0;
        for (int t = 0; t < ntargets; t++) {
            float dx = targets[t].x - cur_x;
            float dy = targets[t].y - cur_y;
            // Converge to target with standard aimbot smooth
            aimbot_converge(dx, dy, 0.8f, 16667, 80);
            cur_x = targets[t].x;
            cur_y = targets[t].y;
            // Brief dwell time on target (100-300ms, simulates shot timing)
            usleep(100000 + (rand() % 200000));
        }
        // Return to origin
        aimbot_converge(-cur_x, -cur_y, 0.8f, 16667, 120);
        usleep(500000);
    }

    trace_stop();
    printf("  ✅ Complete\n");
    trace_write_html("aim_flick");
}

void test_aimbot_recoil(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Recoil Compensation Pattern                          ║\n");
    printf("║  Constant downward pull (recoil table) overlaid with        ║\n");
    printf("║  horizontal micro-corrections (tracking moving target)      ║\n");
    printf("║  Pattern: perfectly regular vertical + small horizontal     ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("aim_recoil");
    usleep(200000);

    // AK-47 style recoil pattern: mostly up, with horizontal wobble
    // Real recoil tables have per-bullet Y values, we simulate a typical spray
    int ak_recoil_y[] = {
        -8,-9,-10,-11,-12,-11,-10,-9,-8,-7,  // Bullets 1-10: strong upward
        -6,-5,-5,-4,-4,-3,-3,-3,-2,-2,        // Bullets 11-20: tapering
        -2,-1,-1,-1,-1, 0, 0, 0, 0, 0         // Bullets 21-30: minimal
    };
    int ak_recoil_x[] = {
         0, 0, 1, 0,-1, 0, 1, 2, 1, 0,       // Bullets 1-10: slight drift
        -1,-2,-3,-2,-1, 0, 1, 2, 3, 2,        // Bullets 11-20: horizontal sweep
         1, 0,-1,-2,-1, 0, 1, 0,-1, 0         // Bullets 21-30: settling
    };
    int nshots = sizeof(ak_recoil_y) / sizeof(ak_recoil_y[0]);

    for (int burst = 0; burst < 6; burst++) {
        printf("  Spray %d/6 (%d bullets)\n", burst + 1, nshots);
        // Fire rate ~600 RPM = 100ms between shots, ~6 frames per bullet at 60Hz
        for (int b = 0; b < nshots; b++) {
            // Each bullet: send recoil compensation over ~6 frames
            int total_dy = ak_recoil_y[b];
            int total_dx = ak_recoil_x[b];
            for (int f = 0; f < 6; f++) {
                int dx = (f == 0) ? total_dx : 0;  // Horizontal on first frame
                int dy = total_dy / 6;              // Spread vertical across frames
                if (f < abs(total_dy) % 6) dy += (total_dy < 0) ? -1 : 1;
                send_move_traced(dx, dy);
                usleep(2800);  // ~6 frames at 60Hz per bullet (~16.8ms)
            }
        }
        // Gap between sprays (reload, reposition)
        usleep(800000);
    }

    trace_stop();
    printf("  ✅ Complete\n");
    trace_write_html("aim_recoil");
}

void test_aimbot_tracking(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Aimbot Tracking (moving target + smooth)             ║\n");
    printf("║  Target moves sinusoidally while aimbot tries to follow     ║\n");
    printf("║  Pattern: proportional controller chasing moving goal       ║\n");
    printf("║  Creates characteristic phase lag + amplitude attenuation   ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("aim_track");
    usleep(200000);

    // Simulate tracking a target that strafes left-right
    float cursor_x = 0, cursor_y = 0;
    float smooth = 0.8f;

    printf("  Tracking slow strafe (10s @ 60Hz)...\n");
    for (int f = 0; f < 600; f++) {
        float t = f / 60.0f;
        // Target position: horizontal strafe + slight vertical bob
        float tgt_x = 150.0f * sinf(2.0f * M_PI * 0.5f * t);  // 0.5Hz, 150px amplitude
        float tgt_y = 30.0f * sinf(2.0f * M_PI * 1.0f * t);   // 1Hz, 30px amplitude

        // Aimbot computation: divide by smooth
        float move_x = (tgt_x - cursor_x) / smooth;
        float move_y = (tgt_y - cursor_y) / smooth;
        int dx = (int)move_x;
        int dy = (int)move_y;
        if (dx > 127) dx = 127; if (dx < -127) dx = -127;
        if (dy > 127) dy = 127; if (dy < -127) dy = -127;

        if (dx != 0 || dy != 0) {
            send_move_traced(dx, dy);
            cursor_x += dx;
            cursor_y += dy;
        }
        usleep(16667);  // 60Hz
    }

    // Phase 2: faster strafe (more lag visible)
    printf("  Tracking fast strafe (5s @ 60Hz)...\n");
    cursor_x = 0; cursor_y = 0;
    for (int f = 0; f < 300; f++) {
        float t = f / 60.0f;
        float tgt_x = 200.0f * sinf(2.0f * M_PI * 2.0f * t);  // 2Hz, bigger
        float tgt_y = 50.0f * cosf(2.0f * M_PI * 1.5f * t);

        float move_x = (tgt_x - cursor_x) / smooth;
        float move_y = (tgt_y - cursor_y) / smooth;
        int dx = (int)move_x;
        int dy = (int)move_y;
        if (dx > 127) dx = 127; if (dx < -127) dx = -127;
        if (dy > 127) dy = 127; if (dy < -127) dy = -127;

        if (dx != 0 || dy != 0) {
            send_move_traced(dx, dy);
            cursor_x += dx;
            cursor_y += dy;
        }
        usleep(16667);
    }

    trace_stop();
    printf("  ✅ Complete\n");
    trace_write_html("aim_track");
}

void test_aimbot_full(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  TEST: Full Aimbot Scenario (engagement simulation)         ║\n");
    printf("║  Combines: flick → track → spray → target switch → spray   ║\n");
    printf("║  This is what a real round looks like to anti-cheat         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("aim_full");
    usleep(200000);

    float cursor_x = 0, cursor_y = 0;

    for (int engagement = 0; engagement < 4; engagement++) {
        printf("  Engagement %d/4\n", engagement + 1);

        // Phase 1: Flick to target (random position)
        float target_x = (float)((rand() % 600) - 300);
        float target_y = (float)((rand() % 300) - 150);
        printf("    Flick to (%.0f, %.0f)\n", target_x, target_y);
        aimbot_converge(target_x - cursor_x, target_y - cursor_y, 0.8f, 16667, 80);
        cursor_x = target_x;
        cursor_y = target_y;

        // Phase 2: Tracking + spray (target strafes while we compensate recoil)
        float track_vx = (float)((rand() % 100) - 50) / 10.0f;
        printf("    Tracking + spray (vx=%.1f)\n", track_vx);
        int bullets = 8 + (rand() % 15);
        for (int b = 0; b < bullets; b++) {
            // Target moves
            target_x += track_vx;
            target_y += (float)((rand() % 10) - 5) * 0.3f;

            // Aimbot tracks + recoil overlay
            float aim_dx = (target_x - cursor_x) / 0.8f;
            float aim_dy = (target_y - cursor_y) / 0.8f;
            int recoil_y = (b < 10) ? -(8 + b) : -(18 - b / 2);

            // 6 sub-frames per bullet
            for (int f = 0; f < 6; f++) {
                int dx = (f == 0) ? (int)aim_dx : 0;
                int dy = recoil_y / 6;
                if (f < abs(recoil_y) % 6) dy += (recoil_y < 0) ? -1 : 1;
                send_move_traced(dx, dy);
                usleep(2800);
            }
            cursor_x += (int)aim_dx;
            cursor_y += (int)aim_dy;
        }

        // Phase 3: Dwell (target eliminated, scanning for next)
        usleep(300000 + (rand() % 500000));
    }

    // Return toward origin
    aimbot_converge(-cursor_x, -cursor_y, 1.5f, 16667, 200);

    trace_stop();
    printf("  ✅ Complete\n");
    trace_write_html("aim_full");
}

// ============================================================================
// Diagnostic Tests - Isolate Individual Humanization Components
//
// These tests are designed so the trace HTML output directly reveals
// which humanization systems are working vs broken. No serial debug
// output needed - the visualization IS the debugger.
//
// HOW TO READ RESULTS:
//
// diag_tremor:  Sends (0,0) moves for 3 seconds. Cursor should NOT move
//               if commands are (0,0). If observed path shows ANY drift,
//               the firmware is adding tremor to zero-delta reports.
//               PASS: obs total dist > 5px, obs jitter > 0.1px
//               FAIL: obs total dist ≈ 0, cursor stationary
//
// diag_line:    Sends identical (5,0) moves at 1kHz for 2 seconds.
//               A perfectly straight horizontal line = no perpendicular
//               jitter being added. Any vertical scatter = jitter alive.
//               PASS: perp scatter > 0.3px, path efficiency < 0.95
//               FAIL: perp scatter ≈ 0, path is ruler-straight
//
// diag_repeat:  Sends constant (3,3) at regular intervals. Checks whether
//               the firmware breaks up identical consecutive deltas.
//               PASS: cmd delta repeats < 80%
//               FAIL: cmd delta repeats ≈ 100%
//
// diag_overshoot: Sends repeated 50px moves with pauses between.
//               Each move should slightly overshoot then correct.
//               PASS: obs path shows small reversals after each move
//               FAIL: obs path stops exactly at target
//
// diag_ease:    Sends a single large move (100,0) and observes the
//               velocity profile. Should show S-curve (slow-fast-slow).
//               PASS: speed variance CV > 0.5
//               FAIL: speed variance CV ≈ 0 (constant speed)
// ============================================================================

void test_diag_tremor(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DIAG: Tremor Isolation (sending 0,0 for 3 seconds)        ║\n");
    printf("║  If firmware adds tremor, cursor will drift visibly.        ║\n");
    printf("║  PASS: obs total dist > 5px   FAIL: cursor stationary      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("diag_tremor");
    usleep(300000);  // 300ms settle

    // Send 3000 zero-delta commands at 1kHz
    // The ONLY way the cursor moves is if firmware adds tremor/jitter
    for (int i = 0; i < 3000; i++) {
        send_move_traced(0, 0);
        usleep(1000);  // 1kHz
    }

    usleep(200000);
    trace_stop();
    printf("  ✅ Check trace: obs total dist > 5px = tremor working\n");
    trace_write_html("diag_tremor");
}

void test_diag_line(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DIAG: Straight Line Jitter (identical 5,0 at 1kHz)        ║\n");
    printf("║  Perfect line = no perpendicular jitter being added.        ║\n");
    printf("║  PASS: perp scatter > 0.3px   FAIL: ruler-straight         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("diag_line");
    usleep(300000);

    // 2000 identical horizontal moves - any vertical deviation = jitter
    for (int i = 0; i < 2000; i++) {
        send_move_traced(5, 0);
        usleep(1000);
    }

    usleep(200000);
    trace_stop();
    printf("  ✅ Check trace: perp scatter > 0.3px = jitter working\n");
    trace_write_html("diag_line");
}

void test_diag_repeat(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DIAG: Delta Repeat Breaking (constant 3,3 at 500Hz)       ║\n");
    printf("║  Firmware should randomize ±1 to break identical deltas.   ║\n");
    printf("║  PASS: delta repeats < 80%%   FAIL: repeats ≈ 100%%         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("diag_repeat");
    usleep(300000);

    // 1000 identical (3,3) commands
    for (int i = 0; i < 1000; i++) {
        send_move_traced(3, 3);
        usleep(2000);  // 500Hz
    }

    usleep(200000);
    trace_stop();
    printf("  ✅ Check trace: cmd delta repeats < 80%% = breaking working\n");
    trace_write_html("diag_repeat");
}

void test_diag_overshoot(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DIAG: Overshoot Detection (50px moves with 500ms gaps)    ║\n");
    printf("║  Each move should slightly overshoot then reverse-correct.  ║\n");
    printf("║  PASS: direction reversals > 10%%   FAIL: no reversals      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("diag_overshoot");
    usleep(300000);

    // 20 discrete moves with gaps - overshoot should be visible in gaps
    for (int i = 0; i < 20; i++) {
        int dir = (i % 2 == 0) ? 1 : -1;
        // Send a single 50px move (firmware should add overshoot + correction)
        send_move_traced(50 * dir, 0);
        usleep(500000);  // 500ms gap - cursor should settle, showing any overshoot
    }

    usleep(200000);
    trace_stop();
    printf("  ✅ Check trace: zoom in on move endings for overshoot bumps\n");
    trace_write_html("diag_overshoot");
}

void test_diag_ease(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DIAG: Easing Curve (single 100px move, observe velocity)  ║\n");
    printf("║  Should show S-curve: slow start, fast middle, slow end.   ║\n");
    printf("║  PASS: speed variance CV > 0.5   FAIL: constant speed      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    trace_start("diag_ease");
    usleep(500000);

    // Single large move - firmware should spread across frames with easing
    send_move_traced(100, 0);
    usleep(1000000);  // 1 second to let it fully execute

    // Repeat with vertical
    send_move_traced(0, 100);
    usleep(1000000);

    // Diagonal
    send_move_traced(70, 70);
    usleep(1000000);

    // And back
    send_move_traced(-100, 0);
    usleep(1000000);
    send_move_traced(0, -100);
    usleep(1000000);
    send_move_traced(-70, -70);
    usleep(1000000);

    trace_stop();
    printf("  ✅ Check trace: hover over points to see speed variation\n");
    trace_write_html("diag_ease");
}

void test_diag_all(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DIAG: Running all diagnostic tests                        ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    test_diag_tremor(); sleep(2);
    test_diag_line(); sleep(2);
    test_diag_repeat(); sleep(2);
    test_diag_overshoot(); sleep(2);
    test_diag_ease();
}

void run_test_mode(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  KMBOX TEST MODE (with trace visualization)                 ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  Mode:   %-50s ║\n", g_config.test_mode);
    printf("║  Port:   %-50s ║\n", g_config.port);
    printf("║  Output: %-50s ║\n", g_config.output_dir);
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    sleep(1);

    if (!strcmp(g_config.test_mode, "rapid")) test_rapid_small_movements();
    else if (!strcmp(g_config.test_mode, "precise")) test_small_precise_movements();
    else if (!strcmp(g_config.test_mode, "flicks")) test_large_fast_flicks();
    else if (!strcmp(g_config.test_mode, "sweep")) test_horizontal_sweep();
    else if (!strcmp(g_config.test_mode, "mixed")) test_mixed_pattern();
    else if (!strcmp(g_config.test_mode, "aim_approach")) test_aimbot_approach();
    else if (!strcmp(g_config.test_mode, "aim_flick")) test_aimbot_flick();
    else if (!strcmp(g_config.test_mode, "aim_recoil")) test_aimbot_recoil();
    else if (!strcmp(g_config.test_mode, "aim_track")) test_aimbot_tracking();
    else if (!strcmp(g_config.test_mode, "aim_full")) test_aimbot_full();
    else if (!strcmp(g_config.test_mode, "diag_tremor")) test_diag_tremor();
    else if (!strcmp(g_config.test_mode, "diag_line")) test_diag_line();
    else if (!strcmp(g_config.test_mode, "diag_repeat")) test_diag_repeat();
    else if (!strcmp(g_config.test_mode, "diag_overshoot")) test_diag_overshoot();
    else if (!strcmp(g_config.test_mode, "diag_ease")) test_diag_ease();
    else if (!strcmp(g_config.test_mode, "diag_all")) test_diag_all();
    else if (!strcmp(g_config.test_mode, "aim_all")) {
        test_aimbot_approach(); sleep(2);
        test_aimbot_flick(); sleep(2);
        test_aimbot_recoil(); sleep(2);
        test_aimbot_tracking(); sleep(2);
        test_aimbot_full();
    }
    else if (!strcmp(g_config.test_mode, "all")) {
        test_rapid_small_movements(); sleep(2);
        test_small_precise_movements(); sleep(2);
        test_large_fast_flicks(); sleep(2);
        test_horizontal_sweep(); sleep(2);
        test_mixed_pattern(); sleep(2);
        test_aimbot_approach(); sleep(2);
        test_aimbot_flick(); sleep(2);
        test_aimbot_recoil(); sleep(2);
        test_aimbot_tracking(); sleep(2);
        test_aimbot_full(); sleep(2);
        test_diag_tremor(); sleep(2);
        test_diag_line(); sleep(2);
        test_diag_repeat(); sleep(2);
        test_diag_overshoot(); sleep(2);
        test_diag_ease();
    } else {
        fprintf(stderr, "Unknown test: %s\nValid: rapid, precise, flicks, sweep, mixed,\n"
                "       aim_approach, aim_flick, aim_recoil, aim_track, aim_full, aim_all,\n"
                "       diag_tremor, diag_line, diag_repeat, diag_overshoot, diag_ease, diag_all,\n"
                "       all\n", g_config.test_mode);
    }

    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DONE - Open trace_*.html files in a browser                ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    if (!parse_args(argc, argv)) return 1;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    srand((unsigned)time(NULL));
    trace_init();

    printf("Opening %s at %d baud...\n", g_config.port, g_config.baudrate);
    g_serial_fd = serial_open(g_config.port, g_config.baudrate);
    if (g_serial_fd < 0) { trace_free(); return 1; }
    printf("Serial port opened.\n");

    g_stats.start_time = time(NULL);
    g_stats.last_response_time = time(NULL);

    if (g_config.test_mode[0]) {
        run_test_mode();
        close(g_serial_fd);
        trace_free();
        return 0;
    }

    // Interactive mode
    if (!setup_event_tap()) { close(g_serial_fd); trace_free(); return 1; }

    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  MOUSE COUNTERACTION ACTIVE                                  ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  Status: %-50s ║\n", g_config.active ? "ENABLED" : "PAUSED");
    printf("║  Gain:   %-50.2f ║\n", g_config.gain);
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  WASD=Move  T=Transform  +/-=Gain  [/]=Step  P=Stats  Q=Quit║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    pthread_create(&g_serial_reader_thread, NULL, serial_reader_thread_fn, NULL);
    pthread_create(&g_serial_sender_thread, NULL, serial_sender_thread_fn, NULL);
    set_terminal_mode(true);

    CFRunLoopTimerRef kt = CFRunLoopTimerCreate(kCFAllocatorDefault, CFAbsoluteTimeGetCurrent()+0.05, 0.05, 0, 0, keyboard_timer_callback, NULL);
    CFRunLoopAddTimer(CFRunLoopGetCurrent(), kt, kCFRunLoopCommonModes);
    CFRunLoopTimerRef st = CFRunLoopTimerCreate(kCFAllocatorDefault, CFAbsoluteTimeGetCurrent()+1.0, 1.0, 0, 0, live_stats_timer_callback, NULL);
    CFRunLoopAddTimer(CFRunLoopGetCurrent(), st, kCFRunLoopCommonModes);

    while (g_config.running) CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, false);

    CFRunLoopTimerInvalidate(kt); CFRelease(kt);
    CFRunLoopTimerInvalidate(st); CFRelease(st);
    pthread_join(g_serial_reader_thread, NULL);
    pthread_join(g_serial_sender_thread, NULL);
    set_terminal_mode(false);
    cleanup_event_tap();
    close(g_serial_fd);
    trace_free();
    print_stats();
    return 0;
}