/**
 * Mouse Movement Counteraction Tool (macOS)
 * 
 * Captures mouse movements via CGEventTap and sends opposite movements
 * to KMBox via serial, effectively locking the cursor.
 * 
 * Usage:
 *   ./mouse_counteract /dev/tty.usbmodem*
 * 
 * Options:
 *   -b, --baud RATE    Serial baud rate (default: 921600)
 *   -g, --gain FLOAT   Counteraction gain 0.0-2.0 (default: 1.0)
 *   -d, --deadzone PX  Ignore movements below this threshold (default: 0)
 *   -v, --verbose      Enable verbose logging
 *   -p, --paused       Start in paused mode
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
 *   S/↓     Move mouse down (hold SHIFT for stats)
 *   D/→     Move mouse right
 *   IJKL    Alternative movement keys
 * 
 * Build:
 *   clang -o mouse_counteract mouse_counteract.c \
 *         -framework CoreGraphics -framework ApplicationServices
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
#include <termios.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include <CoreGraphics/CoreGraphics.h>
#include <ApplicationServices/ApplicationServices.h>

// ============================================================================
// Ferrum Protocol - Text-based commands the Bridge understands
// ============================================================================
// Format: km.move(x, y)\n
// The Bridge translates this to bridge protocol for the KMBox

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
    .baudrate = 921600,
    .gain = 1.0f,
    .deadzone = 0,
    .verbose = false,
    .running = true,
    .active = true
};

// Direct mouse movement settings
static int g_move_step = 20;  // Pixels per keypress
static bool g_transform_enabled = false;  // Whether KMBox transform is enabled

static stats_t g_stats = {0};
static int g_serial_fd = -1;
static CFMachPortRef g_event_tap = NULL;
static CFRunLoopSourceRef g_run_loop_source = NULL;
static pthread_mutex_t g_serial_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_serial_reader_thread;
static pthread_t g_serial_sender_thread;
static volatile bool g_show_live_stats = false;

// Lockless ring buffer for movement commands (callback -> sender thread)
#define MOVE_QUEUE_SIZE 256  // Must be power of 2
#define MOVE_QUEUE_MASK (MOVE_QUEUE_SIZE - 1)
typedef struct {
    int16_t dx;
    int16_t dy;
} move_cmd_t;
static move_cmd_t g_move_queue[MOVE_QUEUE_SIZE];
static volatile uint32_t g_move_queue_head = 0;  // Written by callback
static volatile uint32_t g_move_queue_tail = 0;  // Read by sender thread

// ============================================================================
// Serial Reader Thread - Reads responses from KMBox
// ============================================================================

void* serial_reader_thread(void* arg) {
    (void)arg;
    char buffer[256];
    int buf_pos = 0;
    uint32_t byte_count = 0;
    
    while (g_config.running) {
        char c;
        ssize_t n = read(g_serial_fd, &c, 1);
        
        if (n > 0) {
            byte_count++;
            
            // Debug: print every byte in verbose mode
            if (g_config.verbose && byte_count < 100) {
                printf("[RX BYTE %u] 0x%02X '%c'\n", byte_count, (unsigned char)c, 
                       (c >= 32 && c < 127) ? c : '.');
            }
            
            if (c == '\n' || c == '\r') {
                if (buf_pos > 0) {
                    buffer[buf_pos] = '\0';
                    
                    // Parse response
                    if (strcmp(buffer, "ok") == 0 || strcmp(buffer, ">>>") == 0) {
                        __sync_fetch_and_add(&g_stats.responses_ok, 1);
                        g_stats.last_response_time = time(NULL);
                        if (g_config.verbose) {
                            printf("[RX LINE] OK: '%s'\n", buffer);
                        }
                    } else if (strncmp(buffer, "ERR", 3) == 0) {
                        __sync_fetch_and_add(&g_stats.responses_err, 1);
                        if (g_config.verbose) {
                            printf("[RX LINE] ERROR: '%s'\n", buffer);
                        }
                    } else if (g_config.verbose) {
                        printf("[RX LINE] Unknown: '%s'\n", buffer);
                    }
                    
                    buf_pos = 0;
                }
            } else if (buf_pos < sizeof(buffer) - 1) {
                buffer[buf_pos++] = c;
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            if (g_config.verbose) {
                printf("[RX ERROR] read() failed: %s\n", strerror(errno));
            }
            break;
        } else {
            usleep(1000);  // 1ms sleep if no data
        }
    }
    
    return NULL;
}

// ============================================================================
// Serial Sender Thread - Dequeues movements and sends them
// ============================================================================

void* serial_sender_thread(void* arg) {
    (void)arg;
    uint32_t send_count = 0;
    uint32_t drop_count = 0;
    
    printf("[SENDER] Thread started\n");
    
    while (g_config.running) {
        uint32_t head = g_move_queue_head;
        uint32_t tail = g_move_queue_tail;
        
        if (head != tail) {
            // Coalesce all pending movements into one command
            int32_t total_dx = 0;
            int32_t total_dy = 0;
            int coalesced = 0;
            
            while (head != tail && coalesced < 32) {
                move_cmd_t cmd = g_move_queue[tail & MOVE_QUEUE_MASK];
                total_dx += cmd.dx;
                total_dy += cmd.dy;
                tail = (tail + 1) & MOVE_QUEUE_MASK;
                coalesced++;
            }
            
            // Update tail pointer after coalescing
            __sync_synchronize();
            g_move_queue_tail = tail;
            
            // Clamp to valid range
            if (total_dx > 127) total_dx = 127;
            if (total_dx < -127) total_dx = -127;
            if (total_dy > 127) total_dy = 127;
            if (total_dy < -127) total_dy = -127;
            
            // Send coalesced movement
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "km.move(%d, %d)\n", (int)total_dx, (int)total_dy);
            
            int offset = 0;
            int retries = 0;
            while (offset < len && retries < 100) {
                ssize_t written = write(g_serial_fd, buf + offset, len - offset);
                
                if (written > 0) {
                    offset += written;
                    retries = 0;  // Reset retries on successful write
                } else if (written < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // Buffer full, wait longer and retry
                        usleep(2000);  // 2ms wait for buffer to drain
                        retries++;
                    } else {
                        printf("[SENDER] write() error: %s\n", strerror(errno));
                        __sync_fetch_and_add(&g_stats.errors, 1);
                        break;
                    }
                }
            }
            
            if (offset == len) {
                send_count++;
                __sync_fetch_and_add(&g_stats.packets_sent, 1);
                __sync_fetch_and_add(&g_stats.movements_countered, coalesced);
                if (g_config.verbose && send_count <= 10) {
                    printf("[SENDER] Sent #%u (coalesced %d): km.move(%d, %d)\n", 
                           send_count, coalesced, (int)total_dx, (int)total_dy);
                }
            } else if (retries >= 100) {
                drop_count++;
                if (drop_count <= 5 || drop_count % 100 == 0) {
                    printf("[SENDER] Dropped packet (total drops: %u)\n", drop_count);
                }
            }
            
            // Throttle: ~250 packets/sec max to stay within USB CDC bandwidth
            usleep(4000);
        } else {
            usleep(500);  // 500µs sleep when queue empty
        }
    }
    
    printf("[SENDER] Thread exiting. Total sent: %u, dropped: %u\n", send_count, drop_count);
    return NULL;
}

// ============================================================================
// Serial Functions
// ============================================================================

int serial_open(const char* port, int baudrate) {
    int fd;
    struct termios tty;
    speed_t speed;
    
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", port, strerror(errno));
        return -1;
    }
    
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }
    
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            // For USB CDC, baud rate doesn't matter - it's virtual serial
            speed = B115200;
            break;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1 mode, raw
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    // Keep non-blocking mode for both reads and writes to prevent hangs
    // The O_NONBLOCK was set during open()
    
    return fd;
}

bool send_mouse_move(int fd, int16_t x, int16_t y) {
    // Ferrum protocol: km.move(x, y)\n
    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "km.move(%d, %d)\n", x, y);
    
    pthread_mutex_lock(&g_serial_mutex);
    ssize_t written = write(fd, cmd, len);
    int write_errno = errno;
    pthread_mutex_unlock(&g_serial_mutex);
    
    // Non-blocking write may return -1 with EAGAIN if buffer full
    if (written == len) {
        __sync_fetch_and_add(&g_stats.packets_sent, 1);
        return true;
    }
    
    // If buffer is full, just skip this packet (don't count as error)
    if (written < 0 && (write_errno == EAGAIN || write_errno == EWOULDBLOCK)) {
        // Buffer full - silently drop packet to prevent blocking
        return false;
    }
    
    __sync_fetch_and_add(&g_stats.errors, 1);
    return false;
}

// Send km.transform command to set mouse scaling on KMBox
bool send_transform(int fd, int16_t scale_x, int16_t scale_y, bool enabled) {
    char cmd[64];
    int len = snprintf(cmd, sizeof(cmd), "km.transform(%d, %d, %d)\n", 
                       scale_x, scale_y, enabled ? 1 : 0);
    
    pthread_mutex_lock(&g_serial_mutex);
    ssize_t written = write(fd, cmd, len);
    pthread_mutex_unlock(&g_serial_mutex);
    
    return written == len;
}

// Send direct mouse movement (for keyboard-controlled movement)
bool send_direct_move(int16_t dx, int16_t dy) {
    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "km.move(%d, %d)\n", dx, dy);
    
    pthread_mutex_lock(&g_serial_mutex);
    ssize_t written = write(g_serial_fd, cmd, len);
    pthread_mutex_unlock(&g_serial_mutex);
    
    if (written == len) {
        __sync_fetch_and_add(&g_stats.packets_sent, 1);
        if (g_config.verbose) {
            printf("[DIRECT] Move: %d, %d\n", dx, dy);
        }
        return true;
    }
    return false;
}

// ============================================================================
// CGEventTap Callback - This is where mouse movements are captured
// ============================================================================

CGEventRef mouse_event_callback(CGEventTapProxy proxy, CGEventType type,
                                 CGEventRef event, void* userInfo) {
    (void)proxy;
    (void)userInfo;
    
    // Handle tap being disabled (system can disable it under load)
    if (type == kCGEventTapDisabledByTimeout || 
        type == kCGEventTapDisabledByUserInput) {
        CGEventTapEnable(g_event_tap, true);
        return event;
    }
    
    // Only process mouse movement events
    if (type != kCGEventMouseMoved && 
        type != kCGEventLeftMouseDragged &&
        type != kCGEventRightMouseDragged &&
        type != kCGEventOtherMouseDragged) {
        return event;
    }
    
    // Get mouse delta (relative movement)
    int64_t dx64 = CGEventGetIntegerValueField(event, kCGMouseEventDeltaX);
    int64_t dy64 = CGEventGetIntegerValueField(event, kCGMouseEventDeltaY);
    
    // Skip zero movement
    if (dx64 == 0 && dy64 == 0) {
        return event;
    }
    
    int16_t dx = (int16_t)dx64;
    int16_t dy = (int16_t)dy64;
    
    __sync_fetch_and_add(&g_stats.movements_detected, 1);
    __sync_fetch_and_add(&g_stats.total_dx, labs(dx));
    __sync_fetch_and_add(&g_stats.total_dy, labs(dy));
    
    // Skip if paused
    if (!g_config.active) {
        if (g_config.verbose) {
            printf("[PAUSED] dx=%d dy=%d\n", dx, dy);
        }
        return event;
    }
    
    // Apply deadzone
    if (abs(dx) < g_config.deadzone && abs(dy) < g_config.deadzone) {
        if (g_config.verbose) {
            printf("[DEADZONE] dx=%d dy=%d\n", dx, dy);
        }
        return event;
    }
    
    // Calculate counteraction
    int16_t counter_dx = (int16_t)(-dx * g_config.gain);
    int16_t counter_dy = (int16_t)(-dy * g_config.gain);
    
    // Queue movement for sender thread (lockless enqueue)
    // This is safe because only this callback writes to head
    uint32_t head = g_move_queue_head;
    uint32_t next_head = (head + 1) & MOVE_QUEUE_MASK;
    
    // Check if queue is full (tail is one behind next_head)
    if (next_head != g_move_queue_tail) {
        g_move_queue[head & MOVE_QUEUE_MASK].dx = counter_dx;
        g_move_queue[head & MOVE_QUEUE_MASK].dy = counter_dy;
        __sync_synchronize();  // Memory barrier
        g_move_queue_head = next_head;
    }
    // If queue full, drop the movement silently
    
    return event;
}

// ============================================================================
// Event Tap Setup
// ============================================================================

bool setup_event_tap(void) {
    // Check for accessibility permissions
    if (!AXIsProcessTrusted()) {
        fprintf(stderr, "\n");
        fprintf(stderr, "╔══════════════════════════════════════════════════════════════╗\n");
        fprintf(stderr, "║  ACCESSIBILITY PERMISSION REQUIRED                           ║\n");
        fprintf(stderr, "║                                                              ║\n");
        fprintf(stderr, "║  This app needs permission to monitor mouse events.          ║\n");
        fprintf(stderr, "║                                                              ║\n");
        fprintf(stderr, "║  Go to: System Preferences > Security & Privacy > Privacy    ║\n");
        fprintf(stderr, "║         > Accessibility                                      ║\n");
        fprintf(stderr, "║                                                              ║\n");
        fprintf(stderr, "║  Add this application (or Terminal) to the allowed list.     ║\n");
        fprintf(stderr, "╚══════════════════════════════════════════════════════════════╝\n");
        fprintf(stderr, "\n");
        
        // Prompt for permission (shows system dialog)
        const void* keys[] = { kAXTrustedCheckOptionPrompt };
        const void* values[] = { kCFBooleanTrue };
        CFDictionaryRef options = CFDictionaryCreate(
            kCFAllocatorDefault,
            keys, values, 1,
            &kCFTypeDictionaryKeyCallBacks,
            &kCFTypeDictionaryValueCallBacks
        );
        AXIsProcessTrustedWithOptions(options);
        CFRelease(options);
        
        return false;
    }
    
    // Create event tap for mouse movement events
    CGEventMask event_mask = CGEventMaskBit(kCGEventMouseMoved) |
                             CGEventMaskBit(kCGEventLeftMouseDragged) |
                             CGEventMaskBit(kCGEventRightMouseDragged) |
                             CGEventMaskBit(kCGEventOtherMouseDragged);
    
    g_event_tap = CGEventTapCreate(
        kCGSessionEventTap,           // Tap at session level
        kCGHeadInsertEventTap,        // Insert at head of event stream
        kCGEventTapOptionListenOnly,  // Passive listener (don't block events)
        event_mask,
        mouse_event_callback,
        NULL
    );
    
    if (!g_event_tap) {
        fprintf(stderr, "Error: Failed to create event tap\n");
        return false;
    }
    
    // Create run loop source
    g_run_loop_source = CFMachPortCreateRunLoopSource(kCFAllocatorDefault, 
                                                       g_event_tap, 0);
    if (!g_run_loop_source) {
        fprintf(stderr, "Error: Failed to create run loop source\n");
        CFRelease(g_event_tap);
        g_event_tap = NULL;
        return false;
    }
    
    // Add to current run loop
    CFRunLoopAddSource(CFRunLoopGetCurrent(), g_run_loop_source, 
                       kCFRunLoopCommonModes);
    
    // Enable the tap
    CGEventTapEnable(g_event_tap, true);
    
    return true;
}

void cleanup_event_tap(void) {
    if (g_event_tap) {
        CGEventTapEnable(g_event_tap, false);
    }
    
    if (g_run_loop_source) {
        CFRunLoopRemoveSource(CFRunLoopGetCurrent(), g_run_loop_source,
                              kCFRunLoopCommonModes);
        CFRelease(g_run_loop_source);
        g_run_loop_source = NULL;
    }
    
    if (g_event_tap) {
        CFRelease(g_event_tap);
        g_event_tap = NULL;
    }
}

// ============================================================================
// Statistics & Display
// ============================================================================

void print_stats(void) {
    time_t elapsed = time(NULL) - g_stats.start_time;
    if (elapsed == 0) elapsed = 1;
    
    time_t response_lag = time(NULL) - g_stats.last_response_time;
    
    printf("\n=== Mouse Counteraction Statistics ===\n");
    printf("Runtime:             %ld seconds\n", elapsed);
    printf("Movements detected:  %lld (%.1f/sec)\n", 
           (long long)g_stats.movements_detected,
           (double)g_stats.movements_detected / elapsed);
    printf("Movements countered: %lld\n", (long long)g_stats.movements_countered);
    printf("Total motion:        dx=%lld dy=%lld\n",
           (long long)g_stats.total_dx, (long long)g_stats.total_dy);
    printf("Packets sent:        %lld\n", (long long)g_stats.packets_sent);
    printf("Responses OK:        %lld\n", (long long)g_stats.responses_ok);
    printf("Responses ERR:       %lld\n", (long long)g_stats.responses_err);
    printf("Response lag:        %lds ago\n", response_lag);
    printf("Errors:              %lld\n", (long long)g_stats.errors);
    printf("Current gain:        %.2f\n", g_config.gain);
    printf("======================================\n\n");
}

// ============================================================================
// Terminal & Keyboard Handling
// ============================================================================

static struct termios g_old_term;
static bool g_term_saved = false;

void set_terminal_mode(bool raw) {
    if (raw) {
        struct termios new_term;
        tcgetattr(STDIN_FILENO, &g_old_term);
        g_term_saved = true;
        new_term = g_old_term;
        new_term.c_lflag &= ~(ICANON | ECHO);
        new_term.c_cc[VMIN] = 0;
        new_term.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else {
        if (g_term_saved) {
            tcsetattr(STDIN_FILENO, TCSANOW, &g_old_term);
            fcntl(STDIN_FILENO, F_SETFL, 0);
        }
    }
}

void process_keyboard_input(void) {
    char c;
    if (read(STDIN_FILENO, &c, 1) != 1) {
        return;
    }
    
    // Check for escape sequences (arrow keys)
    if (c == 27) {  // ESC or start of escape sequence
        char seq[2];
        // Try to read more characters (non-blocking)
        if (read(STDIN_FILENO, &seq[0], 1) == 1) {
            if (seq[0] == '[') {
                if (read(STDIN_FILENO, &seq[1], 1) == 1) {
                    switch (seq[1]) {
                        case 'A':  // Up arrow
                            send_direct_move(0, -g_move_step);
                            return;
                        case 'B':  // Down arrow
                            send_direct_move(0, g_move_step);
                            return;
                        case 'C':  // Right arrow
                            send_direct_move(g_move_step, 0);
                            return;
                        case 'D':  // Left arrow
                            send_direct_move(-g_move_step, 0);
                            return;
                    }
                }
            }
        } else {
            // Just ESC key pressed (quit)
            printf("\n[KEYBOARD] Quit key pressed: ESC\n");
            g_config.running = false;
            CFRunLoopStop(CFRunLoopGetCurrent());
            printf("\n>>> Quitting... <<<\n\n");
            return;
        }
    }
    
    switch (c) {
        // ---- Mouse Movement Keys (WASD) ----
        case 'w':
        case 'W':
            send_direct_move(0, -g_move_step);
            break;
        case 'a':
        case 'A':
            send_direct_move(-g_move_step, 0);
            break;
        case 'd':
        case 'D':
            send_direct_move(g_move_step, 0);
            break;
            
        // ---- Alternative Movement Keys (IJKL) ----
        case 'i':
        case 'I':
            send_direct_move(0, -g_move_step);
            break;
        case 'j':
        case 'J':
            send_direct_move(-g_move_step, 0);
            break;
        case 'k':
        case 'K':
            send_direct_move(0, g_move_step);
            break;
        case 'l':
        case 'L':
            send_direct_move(g_move_step, 0);
            break;
            
        // ---- Control Keys ----
        case ' ':
            g_config.active = !g_config.active;
            printf("\n>>> Counteraction %s <<<\n\n", 
                   g_config.active ? "ENABLED" : "DISABLED");
            break;
            
        case 's':  // S moves down (WASD gaming style)
        case 'S':
            send_direct_move(0, g_move_step);
            break;
            
        case 'p':  // Print stats
        case 'P':
        case '?':
            print_stats();
            break;
            
        case 't':
        case 'T':
            g_transform_enabled = !g_transform_enabled;
            if (g_transform_enabled) {
                // Block physical mouse completely
                send_transform(g_serial_fd, 0, 0, true);
                printf("\n>>> Transform: ENABLED (physical mouse blocked) <<<\n\n");
            } else {
                // Restore normal mouse
                send_transform(g_serial_fd, 256, 256, false);
                printf("\n>>> Transform: DISABLED (normal mouse) <<<\n\n");
            }
            break;
            
        case '+':
        case '=':
            g_config.gain = fminf(2.0f, g_config.gain + 0.1f);
            printf("\n>>> Gain: %.2f <<<\n\n", g_config.gain);
            break;
            
        case '-':
        case '_':
            g_config.gain = fmaxf(0.0f, g_config.gain - 0.1f);
            printf("\n>>> Gain: %.2f <<<\n\n", g_config.gain);
            break;
            
        case ']':
            g_move_step = (g_move_step < 200) ? g_move_step + 5 : 200;
            printf("\n>>> Move Step: %d pixels <<<\n\n", g_move_step);
            break;
            
        case '[':
            g_move_step = (g_move_step > 5) ? g_move_step - 5 : 5;
            printf("\n>>> Move Step: %d pixels <<<\n\n", g_move_step);
            break;
            
        case 'q':
        case 'Q':
            printf("\n[KEYBOARD] Quit key pressed: 0x%02X\n", c);
            g_config.running = false;
            CFRunLoopStop(CFRunLoopGetCurrent());
            printf("\n>>> Quitting... <<<\n\n");
            break;
            
        case 'v':
        case 'V':
            g_config.verbose = !g_config.verbose;
            printf("\n>>> Verbose: %s <<<\n\n", 
                   g_config.verbose ? "ON" : "OFF");
            break;
            
        default:
            // Ignore unknown keys, but log them in verbose mode
            if (g_config.verbose) {
                printf("[KEY] Unknown key: 0x%02X '%c'\n", (unsigned char)c, 
                       (c >= 32 && c < 127) ? c : '.');
            }
            break;
    }
}

// ============================================================================
// Run Loop Timer for Keyboard Polling and Live Stats
// ============================================================================

void live_stats_timer_callback(CFRunLoopTimerRef timer, void* info) {
    (void)timer;
    (void)info;
    
    if (!g_show_live_stats || !g_config.running) {
        return;
    }
    
    time_t elapsed = time(NULL) - g_stats.start_time;
    if (elapsed == 0) elapsed = 1;
    time_t response_lag = time(NULL) - g_stats.last_response_time;
    
    printf("\r[LIVE] Movements: %lld | Countered: %lld | OK: %lld | ERR: %lld | Lag: %lds | Rate: %.1f/s    ",
           (long long)g_stats.movements_detected,
           (long long)g_stats.movements_countered,
           (long long)g_stats.responses_ok,
           (long long)g_stats.responses_err,
           response_lag,
           (double)g_stats.movements_detected / elapsed);
    fflush(stdout);
}

void keyboard_timer_callback(CFRunLoopTimerRef timer, void* info) {
    (void)timer;
    (void)info;
    
    if (!g_config.running) {
        CFRunLoopStop(CFRunLoopGetCurrent());
        return;
    }
    
    process_keyboard_input();
}

// ============================================================================
// Signal Handler
// ============================================================================

void signal_handler(int sig) {
    printf("\n[SIGNAL] Received signal %d\n", sig);
    g_config.running = false;
    CFRunLoopStop(CFRunLoopGetMain());
}

// ============================================================================
// Help & Argument Parsing
// ============================================================================

void print_usage(const char* prog) {
    printf("Mouse Movement Counteraction Tool (macOS)\n\n");
    printf("Usage: %s [OPTIONS] <serial_port>\n\n", prog);
    printf("Options:\n");
    printf("  -b, --baud RATE      Serial baud rate (default: 921600)\n");
    printf("  -g, --gain FLOAT     Counteraction gain 0.0-2.0 (default: 1.0)\n");
    printf("  -d, --deadzone PX    Ignore movements below threshold (default: 0)\n");
    printf("  -v, --verbose        Enable verbose logging\n");
    printf("  -p, --paused         Start in paused mode\n");
    printf("  -h, --help           Show this help\n\n");
    printf("Keyboard Controls:\n");
    printf("  SPACE    Toggle counteraction on/off\n");
    printf("  +/-      Adjust gain (±0.1)\n");
    printf("  [/]      Adjust move step (±5 pixels)\n");
    printf("  P/?      Show statistics\n");
    printf("  T        Toggle transform (block physical mouse)\n");
    printf("  V        Toggle verbose mode\n");
    printf("  Q/ESC    Quit\n\n");
    printf("Mouse Movement (Direct Injection):\n");
    printf("  W/↑      Move up\n");
    printf("  A/←      Move left\n");
    printf("  S/↓      Move down\n");
    printf("  D/→      Move right\n");
    printf("  IJKL     Alternative (vim-style)\n\n");
    printf("Examples:\n");
    printf("  %s /dev/tty.usbmodem1234\n", prog);
    printf("  %s -g 1.2 -d 2 -v /dev/tty.usbserial-1234\n\n", prog);
    printf("Note: Requires Accessibility permissions in System Preferences.\n");
}

bool parse_args(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return false;
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            g_config.verbose = true;
        } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--paused") == 0) {
            g_config.active = false;
        } else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "Error: Missing baud rate\n");
                return false;
            }
            g_config.baudrate = atoi(argv[i]);
        } else if (strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--gain") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "Error: Missing gain value\n");
                return false;
            }
            g_config.gain = atof(argv[i]);
            if (g_config.gain < 0.0f || g_config.gain > 2.0f) {
                fprintf(stderr, "Error: Gain must be between 0.0 and 2.0\n");
                return false;
            }
        } else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--deadzone") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "Error: Missing deadzone value\n");
                return false;
            }
            g_config.deadzone = atoi(argv[i]);
        } else if (argv[i][0] != '-') {
            strncpy(g_config.port, argv[i], sizeof(g_config.port) - 1);
        }
    }
    
    if (g_config.port[0] == '\0') {
        fprintf(stderr, "Error: No serial port specified\n\n");
        print_usage(argv[0]);
        return false;
    }
    
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    if (!parse_args(argc, argv)) {
        return 1;
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Open serial port
    printf("Opening %s at %d baud...\n", g_config.port, g_config.baudrate);
    g_serial_fd = serial_open(g_config.port, g_config.baudrate);
    if (g_serial_fd < 0) {
        return 1;
    }
    printf("Serial port opened.\n");
    
    // Setup event tap
    printf("Setting up mouse event capture...\n");
    if (!setup_event_tap()) {
        close(g_serial_fd);
        return 1;
    }
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  MOUSE COUNTERACTION ACTIVE                                  ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  Status:   %-48s ║\n", g_config.active ? "ENABLED" : "PAUSED");
    printf("║  Gain:     %-48.2f ║\n", g_config.gain);
    printf("║  Deadzone: %-48d ║\n", g_config.deadzone);
    printf("║  Verbose:  %-48s ║\n", g_config.verbose ? "ON" : "OFF");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  WASD/Arrows=Move  T=Transform  [/]=Step  P=Stats  Q=Quit   ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    g_stats.start_time = time(NULL);
    g_stats.last_response_time = time(NULL);
    
    // Start serial reader thread
    if (pthread_create(&g_serial_reader_thread, NULL, serial_reader_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create serial reader thread\n");
        close(g_serial_fd);
        cleanup_event_tap();
        return 1;
    }
    
    // Start serial sender thread
    if (pthread_create(&g_serial_sender_thread, NULL, serial_sender_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create serial sender thread\n");
        g_config.running = false;
        pthread_join(g_serial_reader_thread, NULL);
        close(g_serial_fd);
        cleanup_event_tap();
        return 1;
    }
    
    // Setup terminal for keyboard input
    set_terminal_mode(true);
    
    // Create timer for keyboard polling (every 50ms)
    CFRunLoopTimerRef keyboard_timer = CFRunLoopTimerCreate(
        kCFAllocatorDefault,
        CFAbsoluteTimeGetCurrent() + 0.05,  // First fire
        0.05,                                // Interval (50ms)
        0, 0,
        keyboard_timer_callback,
        NULL
    );
    CFRunLoopAddTimer(CFRunLoopGetCurrent(), keyboard_timer, kCFRunLoopCommonModes);
    
    // Create timer for live stats display (every 1 second)
    CFRunLoopTimerRef stats_timer = CFRunLoopTimerCreate(
        kCFAllocatorDefault,
        CFAbsoluteTimeGetCurrent() + 1.0,  // First fire
        1.0,                                // Interval (1 second)
        0, 0,
        live_stats_timer_callback,
        NULL
    );
    CFRunLoopAddTimer(CFRunLoopGetCurrent(), stats_timer, kCFRunLoopCommonModes);
    
    // Run the event loop
    while (g_config.running) {
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, false);
    }
    
    // Cleanup
    printf("Shutting down...\n");
    
    CFRunLoopTimerInvalidate(keyboard_timer);
    CFRelease(keyboard_timer);
    CFRunLoopTimerInvalidate(stats_timer);
    CFRelease(stats_timer);
    
    pthread_join(g_serial_reader_thread, NULL);
    pthread_join(g_serial_sender_thread, NULL);
    
    set_terminal_mode(false);
    cleanup_event_tap();
    close(g_serial_fd);
    
    print_stats();
    
    return 0;
}