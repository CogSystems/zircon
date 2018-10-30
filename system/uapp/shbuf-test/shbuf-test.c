/*
 * Copyright 2018 Cog Systems Pty Ltd. All rights reserved.
 *
 * Use of this source code is governed by the 3-Clause BSD license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef ZIRCON_TOOLCHAIN
#include <zircon/syscalls.h>
#include <zircon/device/link-shbuf.h>
#include <zircon/process.h>
#endif

#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <poll.h>

#ifdef __linux__
#include <linux/ioctl.h>
#include <linux/types.h>
#endif

#ifdef ZIRCON_TOOLCHAIN
#define DEV_SHBUF "/dev/class/link-shbuf/shbuf0"
#else
#define DEV_SHBUF "/dev/okl4-shbuf0"
#define SHBUF_SIZE 0x1000
#endif

#ifdef __linux__
#define OKL4_LINK_SHBUF_IOCTL_IRQ_TX _IOW(0x8d, 1, __u64)
#define OKL4_LINK_SHBUF_IOCTL_IRQ_CLR _IOR(0x8d, 2, __u64)

static ssize_t ioctl_link_shbuf_irq_tx(int fd, uint64_t *payload) {
    return (ssize_t)ioctl(fd, OKL4_LINK_SHBUF_IOCTL_IRQ_TX, payload);
}

static ssize_t ioctl_link_shbuf_irq_clr(int fd, uint64_t *payload) {
    if (ioctl(fd, OKL4_LINK_SHBUF_IOCTL_IRQ_CLR, payload) < 0) {
        return -1;
    }
    return sizeof(uint64_t);
}
#endif

static int usage(char *path) {
    printf("Usage: %s <command> [<arg>]\n", path);
    printf("Commands:\n");
    printf("  read\n");
    printf("  write [string]\n");
    printf("  irq-tx [payload]\n");
    printf("  irq-clr\n");
    printf("  poll [timeout]\n");
    printf("  size\n");
#ifdef ZIRCON_TOOLCHAIN
    printf("  vmo\n");
#endif

    return -1;
}

static int shbuf_read(int fd, char *buf, size_t size) {
    if (read(fd, buf, size) > 0) {
        puts(buf);
    } else {
        printf("Failed to read from %s\n", DEV_SHBUF);
        return -1;
    }
    return 0;
}

static int shbuf_write(int fd, char *buf, size_t size) {
    if (write(fd, buf, size) < 0) {
        printf("Failed to write to %s\n", DEV_SHBUF);
        return -1;
    }
    return 0;
}

static int shbuf_irq_tx(int fd, uint64_t payload) {
    if (ioctl_link_shbuf_irq_tx(fd, &payload) == 0) {
        printf("Sent irq with payload %lu\n", payload);
    } else {
        printf("Failed to send irq\n");
        return -1;
    }
    return 0;
}

static int shbuf_irq_clr(int fd) {
    uint64_t payload;
    if (ioctl_link_shbuf_irq_clr(fd, &payload) == sizeof(uint64_t)) {
        printf("Received irq with payload %lu\n", payload);
    } else {
        printf("Failed to recv irq\n");
        return -1;
    }
    return 0;
}

static int shbuf_poll(int fd, int timeout) {
    int ret;
    struct pollfd pfd = {
        .fd = fd,
        .events = POLLPRI,
        .revents = 0,
    };
    if ((ret = poll(&pfd, 1, timeout)) > 0) {
        printf("Got POLLPRI from %s\n", DEV_SHBUF);
    } else if (ret == 0) {
        printf("Timeout polling %s\n", DEV_SHBUF);
    } else {
        printf("Failed to poll %s\n", DEV_SHBUF);
        return -1;
    }
    return 0;
}

#ifdef ZIRCON_TOOLCHAIN
static int shbuf_vmo(int fd, struct stat *statbuf) {
    zx_handle_t vmo;
    if (ioctl_link_shbuf_get_vmo(fd, &vmo) != sizeof(zx_handle_t)) {
        printf("Failed to get vmo handle\n");
        return -1;
    }
    zx_vm_option_t opt = ZX_VM_PERM_READ | ZX_VM_PERM_WRITE;
    zx_vaddr_t vaddr;
    if (zx_vmar_map(zx_vmar_root_self(), opt,
                0, vmo, 0, statbuf->st_size, &vaddr) != ZX_OK) {
        printf("Failed to map vmo\n");
        return -1;
    }
    printf("Mapped vmo at %p, contents:\n", (void *)vaddr);
    puts((char *)vaddr);
    memset((void *)vaddr, 0, statbuf->st_size);
    printf("Writing to vmo...\n");
    strcpy((char *)vaddr, "VMO TEST STRING");
    return 0;
}
#endif

int main(int argc, char *argv[]) {
    if (argc < 2) {
        return usage(argv[0]);
    }

    int fd = open(DEV_SHBUF, O_RDWR);
    if (fd < 0) {
        printf("Failed to open %s: %s\n",
                DEV_SHBUF, strerror(errno));
        return -1;
    }

    struct stat statbuf = {};
#ifdef ZIRCON_TOOLCHAIN
    if (fstat(fd, &statbuf) < 0) {
        printf("Failed to get stat of %s\n", DEV_SHBUF);
        close(fd);
        return -1;
    }
#else
    statbuf.st_size = SHBUF_SIZE;
#endif

    char *buf = calloc(1, statbuf.st_size);
    if (buf == NULL) {
        printf("Failed to alloc buf\n");
        close(fd);
        return -1;
    }

    int ret;
    if (strcmp(argv[1], "read") == 0) {
        ret = shbuf_read(fd, buf, statbuf.st_size);
    } else if (strcmp(argv[1], "write") == 0) {
        ret = (argc >= 3) ?
                shbuf_write(fd, argv[2], strlen(argv[2])) :
                shbuf_write(fd, buf, statbuf.st_size);
    } else if (strcmp(argv[1], "irq-tx") == 0) {
        uint64_t payload = (argc >= 3) ? strtoul(argv[2], NULL, 10) : 0;
        ret = shbuf_irq_tx(fd, payload);
    } else if (strcmp(argv[1], "irq-clr") == 0) {
        ret = shbuf_irq_clr(fd);
    } else if (strcmp(argv[1], "poll") == 0) {
        int timeout = (argc >= 3) ? atoi(argv[2]) : -1;
        ret = shbuf_poll(fd, timeout);
    } else if (strcmp(argv[1], "size") == 0) {
        printf("Size: %llu\n", (unsigned long long)statbuf.st_size);
        ret = 0;
#ifdef ZIRCON_TOOLCHAIN
    } else if (strcmp(argv[1], "vmo") == 0) {
        ret = shbuf_vmo(fd, &statbuf);
#endif
    } else {
        ret = usage(argv[0]);
    }

    free(buf);
    close(fd);

    return ret;
}
