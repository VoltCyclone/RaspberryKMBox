/**
 * Simple serial echo test - send a command and wait for response
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s /dev/tty.usbmodem*\n", argv[0]);
        return 1;
    }
    
    const char* port = argv[1];
    
    // Open serial port
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", port, strerror(errno));
        return 1;
    }
    
    // Configure serial port
    struct termios tty;
    tcgetattr(fd, &tty);
    
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    
    tty.c_cc[VTIME] = 20;  // 2 second timeout
    tty.c_cc[VMIN] = 0;
    
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    
    printf("Serial port opened: %s\n", port);
    printf("Sending: km.move(10, 10)\\n\n");
    
    // Send command
    const char* cmd = "km.move(10, 10)\n";
    write(fd, cmd, strlen(cmd));
    
    printf("Waiting for response (2 seconds)...\n");
    
    // Read response
    char buffer[256];
    int total = 0;
    int timeout_count = 0;
    
    while (timeout_count < 3) {
        ssize_t n = read(fd, buffer + total, sizeof(buffer) - total - 1);
        if (n > 0) {
            total += n;
            printf("  Received %zd bytes (total: %d)\n", n, total);
            
            // Print hex dump
            printf("  Hex: ");
            for (int i = total - n; i < total; i++) {
                printf("%02X ", (unsigned char)buffer[i]);
            }
            printf("\n");
            
            // Print ASCII
            printf("  ASCII: ");
            for (int i = total - n; i < total; i++) {
                char c = buffer[i];
                printf("%c", (c >= 32 && c < 127) ? c : '.');
            }
            printf("\n");
            
            timeout_count = 0;  // Reset timeout on data
        } else {
            timeout_count++;
            usleep(100000);  // 100ms
        }
    }
    
    if (total == 0) {
        printf("\n❌ NO DATA RECEIVED - Bridge is not responding!\n");
    } else {
        buffer[total] = '\0';
        printf("\n✓ Total received: %d bytes\n", total);
        printf("Raw string: '%s'\n", buffer);
    }
    
    close(fd);
    return (total == 0) ? 1 : 0;
}
