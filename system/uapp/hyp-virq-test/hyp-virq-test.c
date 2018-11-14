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

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#ifdef ZIRCON_TOOLCHAIN
#define VIRQ_DEV "/dev/class/hyp-virq/virq0"
#else
#define VIRQ_DEV "/dev/virq0"
#endif

static int usage(char *path) {
    printf("Usage: %s read\n", path);
    printf("       %s write [payload]\n", path);
    return -1;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        return usage(argv[0]);
    }

    int fd = open(VIRQ_DEV, O_RDWR);
    if (fd < 0) {
        printf("Failed to open %s: %s\n", VIRQ_DEV, strerror(errno));
        return -1;
    }

    uint64_t payload;

    int ret = 0;
    if (strcmp(argv[1], "write") == 0) {
        payload = (argc >= 3) ? strtoul(argv[2], NULL, 10) : 0;
        if (write(fd, &payload, sizeof(payload)) != sizeof(payload)) {
            perror("Write failed");
            ret = -1;
        }
    } else if (strcmp(argv[1], "read") == 0) {
        if (read(fd, &payload, sizeof(payload)) == sizeof(payload)) {
            printf("Payload: %lu\n", payload);
        } else {
            perror("Read failed");
            ret = -1;
        }
    } else {
        ret = usage(argv[0]);
    }

    close(fd);

    return ret;
}
