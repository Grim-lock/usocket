/*-
 *   BSD LICENSE
 *
 *   Copyright (c) 2016, Wei Li
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <unistd.h>
#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <string.h>        // for bzero

#include "../include/usock.h"

static int server_fd = -1;
static int socket_num = 0;

static int connServer(void)
{
    int client_socket = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_in server_addr;

    bzero(&server_addr,sizeof(server_addr));
    server_addr.sin_family = AF_UNIX;
    strcpy(server_addr.sun_path, USER_PROTOCOL_STACK_SOCKET);

    if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        printf("Can Not Connect To Server!\n");
        close(client_socket);
        return -1;
    }

    return client_socket;
}

static void disConnServer()
{
    if (server_fd >= 0) {
        close(server_fd);
        server_fd = -1;
    }
}

int usocket(int protofamily, int type, int protocol)
{
    int fd = -1;
    usock_msg msg;

    (void)protofamily;
    (void)type;
    (void)protocol;

    if (server_fd < 0) {
        server_fd = connServer();
        if (server_fd < 0) {
            return fd;
        }
    }

    bzero(&msg, sizeof(msg));
    msg.msgType = USOCK_MSG_INIT;
    send(server_fd, &msg, sizeof(msg), 0);

    if(recv(server_fd, &msg, sizeof(msg), 0) > 0)
    {
        printf("%s\n", buffer);
        fd = (int)msg.msg.fd;
        break;
    }

    if (fd > 0) {
        socket_num++;
    }

    return fd;
}

void uclose(int fd)
{
    usock_msg msg;

    if (server_fd < 0) {
        return;
    }

    bzero(&msg, sizeof(msg));
    msg.msgType = USOCK_MSG_CLOSE;
    send(server_fd, &msg, sizeof(msg), 0);

    socket_num--;

    if (0 == socket_num) {
        disConnServer();
    }
}

