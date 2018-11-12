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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

int cat(int fd, const char *path) {
    char buf[4096];
    int n;

    while ((n = read(fd, buf, sizeof(buf))) > 0) {
        if (write(STDOUT_FILENO, buf, n) != n) {
            fprintf(stderr, "Failed to write copy of %s: %s\n",
                    path, strerror(errno));
            return -1;
        }
    }

    if (n < 0) {
        fprintf(stderr, "Failed to read %s: %s\n",
                path, strerror(errno));
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[]) {
    int fd;

    if (argc == 1) {
        return cat(STDIN_FILENO, "stdin");
    } else {
        for (int i = 1; i < argc; i++) {
            fd = open(argv[1], O_RDONLY);
            if (fd < 0) {
                fprintf(stderr, "Failed to open %s: %s\n",
                        argv[1], strerror(errno));
                return -1;
            }
            if (cat(fd, argv[1]) != 0) {
                return -1;
            }
        }
    }

    return 0;
}
