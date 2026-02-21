/*
 * KMBox Net UDP Relay — Host-Side Protocol Bridge
 *
 * Speaks KMBox Net protocol over UDP (compatible with KMBox B+ client apps)
 * and translates commands to Wire Protocol v2 binary packets sent over
 * CDC serial to the RaspberryKMBox bridge.
 *
 * Usage:
 *   ./kmbox_relay [options] <serial_port>
 *
 * Options:
 *   -p, --port PORT     UDP listen port (default: 9346)
 *   -b, --baud RATE     Serial baud rate (default: 3000000)
 *   -m, --mac MAC       Expected MAC for auth (default: accept any)
 *   -v, --verbose       Log all commands
 *   -h, --help          Show help
 *
 * Build (macOS/Linux):
 *   cc -O2 -o kmbox_relay kmbox_relay.c
 *
 * Build (Windows, cross-compile with mingw-w64):
 *   x86_64-w64-mingw32-gcc -O2 -o kmbox_relay.exe kmbox_relay.c -lws2_32
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

/* ========================================================================== */
/* Platform Abstraction                                                       */
/* ========================================================================== */

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

typedef SOCKET sock_t;
typedef HANDLE serial_t;
#define INVALID_SOCK INVALID_SOCKET
#define INVALID_SER  INVALID_HANDLE_VALUE
#define sock_close(s) closesocket(s)
#define sock_errno    WSAGetLastError()

static int platform_init(void) {
    WSADATA wsa;
    return WSAStartup(MAKEWORD(2, 2), &wsa);
}
static void platform_cleanup(void) { WSACleanup(); }

static serial_t serial_open(const char *port, int baud) {
    HANDLE h = CreateFileA(port, GENERIC_READ | GENERIC_WRITE,
                           0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) return INVALID_SER;

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) { CloseHandle(h); return INVALID_SER; }
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_ENABLE;
    dcb.fRtsControl  = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX  = FALSE;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); return INVALID_SER; }

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout         = MAXDWORD;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 100;
    SetCommTimeouts(h, &to);
    return h;
}

static void serial_close(serial_t h) {
    if (h != INVALID_SER) CloseHandle(h);
}

static int serial_write(serial_t h, const uint8_t *buf, int len) {
    DWORD written;
    if (!WriteFile(h, buf, len, &written, NULL)) return -1;
    return (int)written;
}

static int serial_read(serial_t h, uint8_t *buf, int maxlen) {
    DWORD rd;
    if (!ReadFile(h, buf, maxlen, &rd, NULL)) return -1;
    return (int)rd;
}

static void set_nonblocking(sock_t s) {
    u_long mode = 1;
    ioctlsocket(s, FIONBIO, &mode);
}

static double time_sec(void) {
    LARGE_INTEGER freq, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart / (double)freq.QuadPart;
}

#else /* POSIX (macOS, Linux) */

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

typedef int sock_t;
typedef int serial_t;
#define INVALID_SOCK (-1)
#define INVALID_SER  (-1)
#define sock_close(s) close(s)
#define sock_errno    errno

static int platform_init(void) { return 0; }
static void platform_cleanup(void) {}

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
#ifdef B460800
        case 460800:  return B460800;
#endif
#ifdef B921600
        case 921600:  return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
#ifdef B3000000
        case 3000000: return B3000000;
#endif
        default:      return B115200;
    }
}

static serial_t serial_open(const char *port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return INVALID_SER;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { close(fd); return INVALID_SER; }

    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return INVALID_SER; }

#ifdef __APPLE__
    /* macOS non-standard baud rate support */
    if (baud > 230400) {
        speed_t custom = baud;
        if (ioctl(fd, IOSSIOSPEED, &custom) == -1) {
            fprintf(stderr, "[WARN] Could not set custom baud %d, using closest standard\n", baud);
        }
    }
#endif

    return fd;
}

static void serial_close(serial_t fd) {
    if (fd != INVALID_SER) close(fd);
}

static int serial_write(serial_t fd, const uint8_t *buf, int len) {
    return (int)write(fd, buf, len);
}

static int serial_read(serial_t fd, uint8_t *buf, int maxlen) {
    int n = (int)read(fd, buf, maxlen);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return n;
}

static void set_nonblocking(sock_t s) {
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
}

static double time_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

#endif /* _WIN32 */

/* ========================================================================== */
/* Wire Protocol v2 (subset needed for relay)                                 */
/* ========================================================================== */

#include "../lib/wire-protocol/include/wire_protocol.h"

/* ========================================================================== */
/* KMBox Net Protocol Structures                                              */
/* ========================================================================== */

#define KMNET_CMD_CONNECT        0xaf3c2828
#define KMNET_CMD_MOUSE_MOVE     0xaede7345
#define KMNET_CMD_MOUSE_LEFT     0x9823AE8D
#define KMNET_CMD_MOUSE_MIDDLE   0x97a3AE8D
#define KMNET_CMD_MOUSE_RIGHT    0x238d8212
#define KMNET_CMD_MOUSE_WHEEL    0xffeead38
#define KMNET_CMD_MOUSE_AUTOMOVE 0xaede7346
#define KMNET_CMD_KEYBOARD_ALL   0x123c2c2f
#define KMNET_CMD_REBOOT         0xaa8855aa
#define KMNET_CMD_BAZER_MOVE     0xa238455a
#define KMNET_CMD_MONITOR        0x27388020
#define KMNET_CMD_DEBUG          0x27382021
#define KMNET_CMD_MASK_MOUSE     0x23234343
#define KMNET_CMD_UNMASK_ALL     0x23344343
#define KMNET_CMD_SETCONFIG      0x1d3d3323
#define KMNET_CMD_SETVIDPID      0xffed3232
#define KMNET_CMD_SHOWPIC        0x12334883
#define KMNET_CMD_TRACE_ENABLE   0xbbcdddac

#pragma pack(push, 1)

typedef struct {
    uint32_t mac;
    uint32_t rand;
    uint32_t indexpts;
    uint32_t cmd;
} kmnet_head_t;

typedef struct {
    int32_t button;
    int32_t x;
    int32_t y;
    int32_t wheel;
    int32_t point[10];
} kmnet_mouse_t;

typedef struct {
    char ctrl;
    char resvel;
    char button[10];
} kmnet_keyboard_t;

#pragma pack(pop)

#define KMNET_HEAD_SIZE    sizeof(kmnet_head_t)        /* 16 */
#define KMNET_MOUSE_SIZE   sizeof(kmnet_mouse_t)       /* 56 */
#define KMNET_KEYBOARD_SIZE sizeof(kmnet_keyboard_t)   /* 12 */
#define KMNET_ENC_SIZE     128                         /* encrypted packet */

/* ========================================================================== */
/* XXTEA Encryption/Decryption                                               */
/* ========================================================================== */

#define XXTEA_DELTA  0x9E3779B9U
#define XXTEA_ROUNDS 6
#define XXTEA_N      32  /* 128 bytes / 4 = 32 uint32_t words */

#define MX(e, p, y, z, sum, key) \
    (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ \
    (((sum) ^ (y)) + ((key)[(p & 3) ^ (e)] ^ (z)))

__attribute__((unused))
static void xxtea_encrypt(uint8_t *data, const uint8_t *key_bytes) {
    uint32_t *v = (uint32_t *)data;
    uint32_t *k = (uint32_t *)key_bytes;
    uint32_t n = XXTEA_N;
    uint32_t z = v[n - 1], y, sum = 0, e;
    uint32_t p, q;

    q = XXTEA_ROUNDS;
    while (q-- > 0) {
        sum += XXTEA_DELTA;
        e = (sum >> 2) & 3;
        for (p = 0; p < n - 1; p++) {
            y = v[p + 1];
            z = v[p] += MX(e, p, y, z, sum, k);
        }
        y = v[0];
        z = v[n - 1] += MX(e, p, y, z, sum, k);
    }
}

static void xxtea_decrypt(uint8_t *data, const uint8_t *key_bytes) {
    uint32_t *v = (uint32_t *)data;
    uint32_t *k = (uint32_t *)key_bytes;
    uint32_t n = XXTEA_N;
    uint32_t z, y = v[0], sum, e;
    uint32_t p, q;

    q = XXTEA_ROUNDS;
    sum = q * XXTEA_DELTA;
    while (q-- > 0) {
        e = (sum >> 2) & 3;
        for (p = n - 1; p > 0; p--) {
            z = v[p - 1];
            y = v[p] -= MX(e, p, y, z, sum, k);
        }
        z = v[n - 1];
        y = v[0] -= MX(e, p, y, z, sum, k);
        sum -= XXTEA_DELTA;
    }
}

/* ========================================================================== */
/* Relay State                                                                */
/* ========================================================================== */

typedef struct {
    /* CLI config */
    char     serial_port[256];
    int      udp_port;
    int      baud_rate;
    uint32_t expected_mac;   /* 0 = accept any */
    bool     verbose;

    /* Runtime */
    serial_t ser;
    sock_t   udp;
    volatile bool running;

    /* Client tracking */
    struct sockaddr_in client_addr;
    bool     client_connected;
    uint32_t client_mac;
    uint8_t  enc_key[16];

    /* Keyboard state tracking (for diffing) */
    kmnet_keyboard_t last_kb;

    /* Button state tracking */
    uint8_t  last_buttons;

    /* Statistics */
    uint64_t cmds_received;
    uint64_t cmds_translated;
    uint64_t serial_errors;
    uint64_t encrypted_cmds;
} relay_t;

static relay_t g_relay;

/* ========================================================================== */
/* Logging                                                                    */
/* ========================================================================== */

static void log_timestamp(void) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(stderr, "%02d:%02d:%02d ", t->tm_hour, t->tm_min, t->tm_sec);
}

#define LOG_BRIDGE(fmt, ...) do { \
    log_timestamp(); \
    fprintf(stderr, "[BRIDGE] " fmt "\n", ##__VA_ARGS__); \
} while(0)

#define LOG_CLIENT(fmt, ...) do { \
    log_timestamp(); \
    fprintf(stderr, "[CLIENT] " fmt "\n", ##__VA_ARGS__); \
} while(0)

#define LOG_VERBOSE(fmt, ...) do { \
    if (g_relay.verbose) { log_timestamp(); fprintf(stderr, "[DEBUG]  " fmt "\n", ##__VA_ARGS__); } \
} while(0)

/* ========================================================================== */
/* Command Name Lookup (for logging)                                          */
/* ========================================================================== */

static const char *cmd_name(uint32_t cmd) {
    switch (cmd) {
        case KMNET_CMD_CONNECT:        return "connect";
        case KMNET_CMD_MOUSE_MOVE:     return "mouse_move";
        case KMNET_CMD_MOUSE_LEFT:     return "mouse_left";
        case KMNET_CMD_MOUSE_MIDDLE:   return "mouse_middle";
        case KMNET_CMD_MOUSE_RIGHT:    return "mouse_right";
        case KMNET_CMD_MOUSE_WHEEL:    return "mouse_wheel";
        case KMNET_CMD_MOUSE_AUTOMOVE: return "mouse_automove";
        case KMNET_CMD_KEYBOARD_ALL:   return "keyboard_all";
        case KMNET_CMD_REBOOT:         return "reboot";
        case KMNET_CMD_BAZER_MOVE:     return "bezier_move";
        case KMNET_CMD_MONITOR:        return "monitor";
        case KMNET_CMD_DEBUG:          return "debug";
        case KMNET_CMD_MASK_MOUSE:     return "mask_mouse";
        case KMNET_CMD_UNMASK_ALL:     return "unmask_all";
        case KMNET_CMD_SETCONFIG:      return "setconfig";
        case KMNET_CMD_SETVIDPID:      return "setvidpid";
        case KMNET_CMD_SHOWPIC:        return "showpic";
        case KMNET_CMD_TRACE_ENABLE:   return "trace_enable";
        default:                       return "unknown";
    }
}

/* ========================================================================== */
/* Serial Helpers                                                             */
/* ========================================================================== */

static int send_wire(relay_t *r, const uint8_t *pkt, int len) {
    int written = serial_write(r->ser, pkt, len);
    if (written != len) {
        r->serial_errors++;
        LOG_BRIDGE("Serial write error: wrote %d/%d", written, len);
        return -1;
    }
    return 0;
}

/* ========================================================================== */
/* UDP Helpers                                                                */
/* ========================================================================== */

static void send_ack(relay_t *r, const kmnet_head_t *head) {
    sendto(r->udp, (const char *)head, KMNET_HEAD_SIZE, 0,
           (struct sockaddr *)&r->client_addr, sizeof(r->client_addr));
}

/* ========================================================================== */
/* Encryption Key Derivation                                                  */
/* ========================================================================== */

static void derive_key(relay_t *r, uint32_t mac) {
    memset(r->enc_key, 0, 16);
    r->enc_key[0] = (mac >> 24) & 0xFF;
    r->enc_key[1] = (mac >> 16) & 0xFF;
    r->enc_key[2] = (mac >> 8)  & 0xFF;
    r->enc_key[3] = (mac >> 0)  & 0xFF;
}

/* ========================================================================== */
/* Command Translation: KMBox Net -> Wire Protocol v2                         */
/* ========================================================================== */

static void translate_mouse_move(relay_t *r, const kmnet_mouse_t *m) {
    uint8_t pkt[WIRE_MAX_PACKET];
    int16_t x = (int16_t)m->x;
    int16_t y = (int16_t)m->y;
    size_t len = wire_build_move(pkt, x, y);
    LOG_VERBOSE("mouse_move x=%d y=%d (wire=%s, %zu bytes)",
                x, y, len == 3 ? "MOVE8" : "MOVE16", len);
    send_wire(r, pkt, (int)len);
    r->cmds_translated++;
}

static void translate_mouse_button(relay_t *r, const kmnet_mouse_t *m) {
    /* KMBox Net button bits: 0x01=left, 0x02=right, 0x04=middle, 0x08=side1, 0x10=side2
     * Wire protocol button bits match: 0x01=left, 0x02=right, 0x04=middle, 0x08=back, 0x10=forward */
    uint8_t buttons = (uint8_t)(m->button & 0x1F);
    if (buttons != r->last_buttons) {
        uint8_t pkt[WIRE_MAX_PACKET];
        size_t len = wire_build_buttons(pkt, buttons);
        LOG_VERBOSE("buttons 0x%02X -> 0x%02X", r->last_buttons, buttons);
        send_wire(r, pkt, (int)len);
        r->last_buttons = buttons;
        r->cmds_translated++;
    }
}

static void translate_mouse_wheel(relay_t *r, const kmnet_mouse_t *m) {
    uint8_t pkt[WIRE_MAX_PACKET];
    int8_t delta = (int8_t)m->wheel;
    size_t len = wire_build_wheel(pkt, delta);
    LOG_VERBOSE("wheel delta=%d", delta);
    send_wire(r, pkt, (int)len);
    r->cmds_translated++;
}

static void translate_mouse_automove(relay_t *r, const kmnet_mouse_t *m,
                                      uint32_t duration_ms) {
    uint8_t pkt[WIRE_MAX_PACKET];
    int16_t x = (int16_t)m->x;
    int16_t y = (int16_t)m->y;
    /* Use smooth injection mode; rand field carries duration */
    (void)duration_ms;
    size_t len = wire_build_smooth16(pkt, x, y, WIRE_INJECT_SMOOTH);
    LOG_VERBOSE("automove x=%d y=%d duration=%ums", x, y, duration_ms);
    send_wire(r, pkt, (int)len);
    r->cmds_translated++;
}

static void translate_bezier_move(relay_t *r, const kmnet_mouse_t *m,
                                   uint32_t duration_ms) {
    uint8_t pkt[WIRE_MAX_PACKET];
    int16_t x = (int16_t)m->x;
    int16_t y = (int16_t)m->y;
    (void)duration_ms;
    size_t len = wire_build_smooth16(pkt, x, y, WIRE_INJECT_SMOOTH);
    LOG_VERBOSE("bezier x=%d y=%d duration=%ums", x, y, duration_ms);
    send_wire(r, pkt, (int)len);
    r->cmds_translated++;
}

static void translate_keyboard(relay_t *r, const kmnet_keyboard_t *kb) {
    uint8_t pkt[WIRE_MAX_PACKET];

    /* Diff modifiers: send keydown/keyup for changed modifier bits */
    uint8_t old_ctrl = (uint8_t)r->last_kb.ctrl;
    uint8_t new_ctrl = (uint8_t)kb->ctrl;
    if (old_ctrl != new_ctrl) {
        /* Modifier bits map to HID usage codes 0xE0..0xE7 */
        for (int bit = 0; bit < 8; bit++) {
            uint8_t mask = 1 << bit;
            bool was = (old_ctrl & mask) != 0;
            bool now = (new_ctrl & mask) != 0;
            if (!was && now) {
                size_t len = wire_build_keydown(pkt, 0xE0 + bit, 0);
                LOG_VERBOSE("keydown modifier 0x%02X", 0xE0 + bit);
                send_wire(r, pkt, (int)len);
                r->cmds_translated++;
            } else if (was && !now) {
                size_t len = wire_build_keyup(pkt, 0xE0 + bit);
                LOG_VERBOSE("keyup modifier 0x%02X", 0xE0 + bit);
                send_wire(r, pkt, (int)len);
                r->cmds_translated++;
            }
        }
    }

    /* Diff regular keys */
    /* Keys released: in old but not in new */
    for (int i = 0; i < 10; i++) {
        uint8_t old_key = (uint8_t)r->last_kb.button[i];
        if (old_key == 0) continue;
        bool found = false;
        for (int j = 0; j < 10; j++) {
            if ((uint8_t)kb->button[j] == old_key) { found = true; break; }
        }
        if (!found) {
            size_t len = wire_build_keyup(pkt, old_key);
            LOG_VERBOSE("keyup 0x%02X", old_key);
            send_wire(r, pkt, (int)len);
            r->cmds_translated++;
        }
    }

    /* Keys pressed: in new but not in old */
    for (int i = 0; i < 10; i++) {
        uint8_t new_key = (uint8_t)kb->button[i];
        if (new_key == 0) continue;
        bool found = false;
        for (int j = 0; j < 10; j++) {
            if ((uint8_t)r->last_kb.button[j] == new_key) { found = true; break; }
        }
        if (!found) {
            /* Send keydown with current modifier state */
            size_t len = wire_build_keydown(pkt, new_key, new_ctrl);
            LOG_VERBOSE("keydown 0x%02X (mods=0x%02X)", new_key, new_ctrl);
            send_wire(r, pkt, (int)len);
            r->cmds_translated++;
        }
    }

    memcpy(&r->last_kb, kb, sizeof(kmnet_keyboard_t));
}

/* ========================================================================== */
/* Packet Handler                                                             */
/* ========================================================================== */

static void handle_packet(relay_t *r, uint8_t *buf, int len,
                           struct sockaddr_in *from) {
    bool encrypted = false;
    uint8_t decrypted[KMNET_ENC_SIZE];

    /* Detect encrypted packet: always exactly 128 bytes */
    if (len == KMNET_ENC_SIZE) {
        /* Try decryption if we have a key */
        if (r->client_connected) {
            memcpy(decrypted, buf, KMNET_ENC_SIZE);
            xxtea_decrypt(decrypted, r->enc_key);
            buf = decrypted;
            encrypted = true;
            r->encrypted_cmds++;
        }
    }

    if (len < (int)KMNET_HEAD_SIZE && !encrypted) {
        LOG_VERBOSE("Packet too short: %d bytes", len);
        return;
    }

    kmnet_head_t head;
    memcpy(&head, buf, KMNET_HEAD_SIZE);

    r->cmds_received++;

    /* Handle connect — always process regardless of MAC filter */
    if (head.cmd == KMNET_CMD_CONNECT) {
        r->client_mac = head.mac;
        memcpy(&r->client_addr, from, sizeof(*from));
        r->client_connected = true;
        derive_key(r, head.mac);
        memset(&r->last_kb, 0, sizeof(r->last_kb));
        r->last_buttons = 0;

        LOG_CLIENT("Connected from %s:%d (MAC: %08X)",
                   inet_ntoa(from->sin_addr), ntohs(from->sin_port), head.mac);

        if (r->expected_mac != 0 && head.mac != r->expected_mac) {
            LOG_CLIENT("WARNING: MAC mismatch (expected %08X)", r->expected_mac);
        }

        send_ack(r, &head);
        return;
    }

    /* For non-connect commands, must have a connected client */
    if (!r->client_connected) {
        LOG_VERBOSE("Ignoring %s: no client connected", cmd_name(head.cmd));
        return;
    }

    /* Update client address (in case port changed) */
    memcpy(&r->client_addr, from, sizeof(*from));

    LOG_VERBOSE("cmd=%s (0x%08X) indexpts=%u rand=%u %s",
                cmd_name(head.cmd), head.cmd, head.indexpts, head.rand,
                encrypted ? "[encrypted]" : "");

    /* Extract payloads */
    const uint8_t *payload = buf + KMNET_HEAD_SIZE;
    int payload_len = (encrypted ? KMNET_ENC_SIZE : len) - (int)KMNET_HEAD_SIZE;

    switch (head.cmd) {
    case KMNET_CMD_MOUSE_MOVE:
        if (payload_len >= (int)KMNET_MOUSE_SIZE) {
            kmnet_mouse_t m;
            memcpy(&m, payload, KMNET_MOUSE_SIZE);
            translate_mouse_move(r, &m);
        }
        break;

    case KMNET_CMD_MOUSE_LEFT:
    case KMNET_CMD_MOUSE_RIGHT:
    case KMNET_CMD_MOUSE_MIDDLE:
        if (payload_len >= (int)KMNET_MOUSE_SIZE) {
            kmnet_mouse_t m;
            memcpy(&m, payload, KMNET_MOUSE_SIZE);
            translate_mouse_button(r, &m);
        }
        break;

    case KMNET_CMD_MOUSE_WHEEL:
        if (payload_len >= (int)KMNET_MOUSE_SIZE) {
            kmnet_mouse_t m;
            memcpy(&m, payload, KMNET_MOUSE_SIZE);
            translate_mouse_wheel(r, &m);
        }
        break;

    case KMNET_CMD_MOUSE_AUTOMOVE:
        if (payload_len >= (int)KMNET_MOUSE_SIZE) {
            kmnet_mouse_t m;
            memcpy(&m, payload, KMNET_MOUSE_SIZE);
            translate_mouse_automove(r, &m, head.rand);
        }
        break;

    case KMNET_CMD_BAZER_MOVE:
        if (payload_len >= (int)KMNET_MOUSE_SIZE) {
            kmnet_mouse_t m;
            memcpy(&m, payload, KMNET_MOUSE_SIZE);
            translate_bezier_move(r, &m, head.rand);
        }
        break;

    case KMNET_CMD_KEYBOARD_ALL:
        if (payload_len >= (int)KMNET_KEYBOARD_SIZE) {
            kmnet_keyboard_t kb;
            memcpy(&kb, payload, KMNET_KEYBOARD_SIZE);
            translate_keyboard(r, &kb);
        }
        break;

    case KMNET_CMD_REBOOT: {
        uint8_t pkt[WIRE_MAX_PACKET];
        size_t plen = wire_build_ping(pkt);
        LOG_VERBOSE("reboot -> ping");
        send_wire(r, pkt, (int)plen);
        r->cmds_translated++;
        break;
    }

    case KMNET_CMD_MONITOR:
        LOG_VERBOSE("monitor command (port=%d) — not fully supported over serial",
                    head.rand & 0xFFFF);
        break;

    default:
        LOG_VERBOSE("unsupported command 0x%08X — ACK only", head.cmd);
        break;
    }

    /* Always ACK */
    send_ack(r, &head);
}

/* ========================================================================== */
/* Main Poll Loop                                                             */
/* ========================================================================== */

static void signal_handler(int sig) {
    (void)sig;
    g_relay.running = false;
}

static void run_relay(relay_t *r) {
    uint8_t udp_buf[2048];
    uint8_t ser_buf[256];
    double last_stats = time_sec();

    LOG_BRIDGE("Relay running — UDP port %d, serial %s @ %d baud",
               r->udp_port, r->serial_port, r->baud_rate);
    LOG_BRIDGE("Waiting for KMBox Net client connection...");

    while (r->running) {
        fd_set rfds;
        struct timeval tv;
        int maxfd = 0;

        FD_ZERO(&rfds);

#ifdef _WIN32
        FD_SET(r->udp, &rfds);
        maxfd = 0; /* Windows ignores nfds in select() */
#else
        FD_SET(r->udp, &rfds);
        if (r->udp > maxfd) maxfd = r->udp;
        FD_SET(r->ser, &rfds);
        if (r->ser > maxfd) maxfd = r->ser;
#endif

        tv.tv_sec = 0;
        tv.tv_usec = 1000; /* 1ms poll */

        int ready = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            LOG_BRIDGE("select() error: %s", strerror(errno));
            break;
        }

        /* Check UDP socket */
        if (FD_ISSET(r->udp, &rfds)) {
            struct sockaddr_in from;
            socklen_t fromlen = sizeof(from);
            int n = recvfrom(r->udp, (char *)udp_buf, sizeof(udp_buf), 0,
                             (struct sockaddr *)&from, &fromlen);
            if (n > 0) {
                handle_packet(r, udp_buf, n, &from);
            }
        }

        /* Check serial port for responses (drain buffer) */
#ifdef _WIN32
        {
            int n = serial_read(r->ser, ser_buf, sizeof(ser_buf));
            if (n > 0) {
                LOG_VERBOSE("Serial rx: %d bytes", n);
            }
        }
#else
        if (FD_ISSET(r->ser, &rfds)) {
            int n = serial_read(r->ser, ser_buf, sizeof(ser_buf));
            if (n > 0) {
                LOG_VERBOSE("Serial rx: %d bytes", n);
            } else if (n < 0) {
                LOG_BRIDGE("Serial read error: %s", strerror(errno));
            }
        }
#endif

        /* Periodic stats (every 30s) */
        double now = time_sec();
        if (now - last_stats > 30.0) {
            LOG_BRIDGE("Stats: %llu cmds recv, %llu translated, %llu encrypted, %llu serial errors",
                       (unsigned long long)r->cmds_received,
                       (unsigned long long)r->cmds_translated,
                       (unsigned long long)r->encrypted_cmds,
                       (unsigned long long)r->serial_errors);
            last_stats = now;
        }
    }

    LOG_BRIDGE("Shutting down...");
}

/* ========================================================================== */
/* CLI Argument Parsing                                                       */
/* ========================================================================== */

static void usage(const char *prog) {
    fprintf(stderr,
        "KMBox Net UDP Relay — Host-Side Protocol Bridge\n"
        "\n"
        "Usage: %s [options] <serial_port>\n"
        "\n"
        "Options:\n"
        "  -p, --port PORT     UDP listen port (default: 9346)\n"
        "  -b, --baud RATE     Serial baud rate (default: 3000000)\n"
        "  -m, --mac MAC       Expected MAC for auth (hex, default: accept any)\n"
        "  -v, --verbose       Log all commands\n"
        "  -h, --help          Show help\n"
        "\n"
        "Example:\n"
        "  %s -v /dev/tty.usbmodem2101\n"
        "  %s -p 16896 -m AABBCCDD COM3\n"
        "\n"
        "Client connection:\n"
        "  kmNet_init(\"127.0.0.1\", \"9346\", \"AABBCCDD\")\n",
        prog, prog, prog);
}

static uint32_t parse_mac(const char *s) {
    uint32_t mac = 0;
    for (int i = 0; i < 8 && s[i]; i++) {
        char c = s[i];
        uint8_t nibble;
        if (c >= '0' && c <= '9')      nibble = c - '0';
        else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
        else break;
        mac = (mac << 4) | nibble;
    }
    return mac;
}

static int parse_args(relay_t *r, int argc, char **argv) {
    r->udp_port     = 9346;
    r->baud_rate    = 3000000;
    r->expected_mac = 0;
    r->verbose      = false;
    r->serial_port[0] = '\0';

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return -1;
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            r->verbose = true;
        } else if ((strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) && i + 1 < argc) {
            r->udp_port = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) && i + 1 < argc) {
            r->baud_rate = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mac") == 0) && i + 1 < argc) {
            r->expected_mac = parse_mac(argv[++i]);
        } else if (argv[i][0] != '-') {
            strncpy(r->serial_port, argv[i], sizeof(r->serial_port) - 1);
            r->serial_port[sizeof(r->serial_port) - 1] = '\0';
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return -1;
        }
    }

    if (r->serial_port[0] == '\0') {
        fprintf(stderr, "Error: serial port required\n\n");
        usage(argv[0]);
        return -1;
    }

    return 0;
}

/* ========================================================================== */
/* Main                                                                       */
/* ========================================================================== */

int main(int argc, char **argv) {
    relay_t *r = &g_relay;
    memset(r, 0, sizeof(*r));
    r->ser = INVALID_SER;
    r->udp = INVALID_SOCK;

    if (parse_args(r, argc, argv) != 0)
        return 1;

    if (platform_init() != 0) {
        fprintf(stderr, "Platform init failed\n");
        return 1;
    }

    /* Open serial port */
    r->ser = serial_open(r->serial_port, r->baud_rate);
    if (r->ser == INVALID_SER) {
        fprintf(stderr, "Failed to open serial port %s: %s\n",
                r->serial_port, strerror(errno));
        platform_cleanup();
        return 1;
    }
    LOG_BRIDGE("Connected to %s @ %d baud", r->serial_port, r->baud_rate);

    /* Create UDP socket */
    r->udp = socket(AF_INET, SOCK_DGRAM, 0);
    if (r->udp == INVALID_SOCK) {
        fprintf(stderr, "Failed to create UDP socket\n");
        serial_close(r->ser);
        platform_cleanup();
        return 1;
    }

    /* Allow address reuse */
    int reuse = 1;
    setsockopt(r->udp, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    /* Bind */
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port        = htons((uint16_t)r->udp_port);

    if (bind(r->udp, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        fprintf(stderr, "Failed to bind UDP port %d: %s\n",
                r->udp_port, strerror(errno));
        sock_close(r->udp);
        serial_close(r->ser);
        platform_cleanup();
        return 1;
    }

    set_nonblocking(r->udp);

    /* Install signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    r->running = true;
    run_relay(r);

    /* Cleanup */
    sock_close(r->udp);
    serial_close(r->ser);
    platform_cleanup();

    LOG_BRIDGE("Final stats: %llu cmds recv, %llu translated, %llu encrypted, %llu serial errors",
               (unsigned long long)r->cmds_received,
               (unsigned long long)r->cmds_translated,
               (unsigned long long)r->encrypted_cmds,
               (unsigned long long)r->serial_errors);

    return 0;
}
