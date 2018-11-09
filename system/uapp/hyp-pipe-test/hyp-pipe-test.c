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
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#ifdef ZIRCON_TOOLCHAIN
#define PIPE_DEV "/dev/class/hyp-pipe/pipe0"
#else
#define PIPE_DEV "/dev/okl4-pipe0"
#endif

static int usage(char *path) {
    printf("Usage: %s (read|write) [count]\n", path);
    return -1;
}

int main(int argc, char *argv[]) {
    char c;
    int count = 10;
    int ret;
    bool do_write;

    if (argc < 2) {
        return usage(argv[0]);
    }

    if (strcmp(argv[1], "write") == 0) {
        do_write = true;
    } else if (strcmp(argv[1], "read") == 0) {
        do_write = false;
    } else {
        return usage(argv[0]);
    }

    if (argc >= 3) {
        count = atoi(argv[2]);
    }

    int fd = open(PIPE_DEV, O_RDWR);
    if (fd < 0) {
        printf("Failed to open %s: %s\n",
                PIPE_DEV, strerror(errno));
        return -1;
    }

    while (count > 0) {
        if (do_write) {
            c = getchar();
            ret = write(fd, &c, 1);
            if (ret < 0) {
                perror("Write failed");
                break;
            }
            putchar(c);
            fflush(stdout);
        } else {
            ret = read(fd, &c, 1);
            if (ret < 0) {
                perror("Read failed");
                break;
            }
            putchar(c);
        }
        count--;
    }

    close(fd);

    return 0;
}
