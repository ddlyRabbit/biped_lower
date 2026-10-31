/*
 * simcan.c — LD_PRELOAD shim that turns SocketCAN sockets into UNIX SEQPACKET sockets.
 *
 * The robot's drivers open PF_CAN/SOCK_RAW sockets, look the interface up with
 * ioctl(SIOCGIFINDEX) and bind to it.  Under this shim:
 *   socket(PF_CAN, ...)        -> socket(AF_UNIX, SOCK_SEQPACKET, 0)   (fd remembered)
 *   ioctl(fd, SIOCGIFINDEX, r) -> connect(fd, "$SIMCAN_DIR/<ifname>")  (the motor emulator)
 *   bind / setsockopt(SOL_CAN_RAW | timestamps) on that fd -> success, no-op
 * Every can_frame (16 bytes) the driver writes becomes one datagram to the emulator and
 * every datagram the emulator sends reads back as one can_frame, so read/write/poll/recv/send
 * work unchanged.  Sockets of any other family are untouched.
 *
 * Build: cc -O2 -shared -fPIC -o libsimcan.so simcan.c -ldl
 * MIT License, Copyright (c) 2026 Abhinav Roy
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <net/if.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#ifndef PF_CAN
#define PF_CAN 29
#endif
#ifndef SOL_CAN_BASE
#define SOL_CAN_BASE 100
#endif
#define MAXFD 4096

static unsigned char fake[MAXFD];

static int (*real_socket)(int, int, int);
static int (*real_ioctl)(int, unsigned long, ...);
static int (*real_bind)(int, const struct sockaddr *, socklen_t);
static int (*real_setsockopt)(int, int, int, const void *, socklen_t);
static int (*real_close)(int);

static void init(void) {
    if (real_socket) return;
    real_socket = dlsym(RTLD_NEXT, "socket");
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    real_bind = dlsym(RTLD_NEXT, "bind");
    real_setsockopt = dlsym(RTLD_NEXT, "setsockopt");
    real_close = dlsym(RTLD_NEXT, "close");
}

int socket(int domain, int type, int protocol) {
    init();
    if (domain != PF_CAN) return real_socket(domain, type, protocol);
    int fd = real_socket(AF_UNIX, SOCK_SEQPACKET | (type & (SOCK_NONBLOCK | SOCK_CLOEXEC)), 0);
    if (fd >= 0 && fd < MAXFD) fake[fd] = 1;
    return fd;
}

static int attach(int fd, const char *ifname) {
    const char *dir = getenv("SIMCAN_DIR");
    struct sockaddr_un a;
    memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s/%s", dir ? dir : "/tmp/simcan", ifname);
    if (connect(fd, (struct sockaddr *)&a, sizeof a) < 0) { errno = ENODEV; return -1; }
    return 0;
}

int ioctl(int fd, unsigned long req, ...) {
    init();
    va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);
    if (fd >= 0 && fd < MAXFD && fake[fd] && req == SIOCGIFINDEX) {
        struct ifreq *r = (struct ifreq *)arg;
        if (attach(fd, r->ifr_name) < 0) return -1;
        r->ifr_ifindex = 1000 + (unsigned char)r->ifr_name[strlen(r->ifr_name) - 1];
        return 0;
    }
    return real_ioctl(fd, req, arg);
}

int bind(int fd, const struct sockaddr *addr, socklen_t len) {
    init();
    if (fd >= 0 && fd < MAXFD && fake[fd]) return 0;
    return real_bind(fd, addr, len);
}

int setsockopt(int fd, int level, int name, const void *val, socklen_t len) {
    init();
    if (fd >= 0 && fd < MAXFD && fake[fd] && (level >= SOL_CAN_BASE || level == SOL_SOCKET)) return 0;
    return real_setsockopt(fd, level, name, val, len);
}

int close(int fd) {
    init();
    if (fd >= 0 && fd < MAXFD) fake[fd] = 0;
    return real_close(fd);
}
