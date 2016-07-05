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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <sys/time.h>
#include <netinet/in.h>

#include "../include/usock.h"

typedef struct _CLIENT {
    int fd;
    struct sockaddr_in addr; /* client's address information */
} CLIENT;

#define MAX_EVENTS 500

int currentClient = 0;

#define REVLEN 1024
char recvBuf[REVLEN];

static uint32_t listenEvents = EPOLLIN | EPOLLET | EPOLLRDHUP;
static uint32_t clientEvents = EPOLLIN | EPOLLET | EPOLLRDHUP;
static int epollfd;
struct epoll_event eventList[MAX_EVENTS];

static void AcceptConn(int srvfd)
{
    struct sockaddr_in sin;
    socklen_t len = sizeof(struct sockaddr_in);

    bzero(&sin, len);
    int confd = accept(srvfd, (struct sockaddr*)&sin, &len);

    if (confd < 0) {
        printf("bad accept\n");
        return;
    } else {
        printf("Accept Connection: %d", confd);
    }

    //setnonblocking(confd);

    struct epoll_event event;
    event.data.fd = confd;
    event.events =  clientEvents;
    epoll_ctl(epollfd, EPOLL_CTL_ADD, confd, &event);
}

static void RecvData(int fd)
{
    int ret;
    int recvLen = 0;
    usock_msg msg;

    memset(recvBuf, 0, REVLEN);
    printf("RecvData function\n");

    while(1) {
        ret = recv(fd, (char *)recvBuf + recvLen, REVLEN - recvLen, 0);
        if(ret == 0) {
            recvLen = 0;
            break;
        } else if(ret < 0) {
            recvLen = 0;
            break;
        }

        recvLen = recvLen + ret;
        if(recvLen < REVLEN) {
            continue;
        } else {
            printf("buf = %s\n",  recvBuf);
            recvLen = 0;
            break;
        }
    }

    if (recvLen > 0) {
        assert(recvLen == sizeof(usock_msg));
        memcpy(&msg, recvBuf, recvLen);
        switch(msg.msgType) {
            case USOCK_MSG_INIT: {
                printf("init usock");
                usocket *usocket = usocket_create();
                send(fd, usocket.eventFd, sizeof(usocket.eventFd));
                break;
            }
            case USOCK_MSG_CLOSE: {
                usocket_delete(msg.msg.fd);
                printf("close usock");
                break;
            }
            default: {
                assert(0);
            }
        }
    }
}

int main(int argc, char **argv)
{
    int sockListen;
    struct sockaddr_un server_addr;

    (void)argc;
    (void)argv;

    memset(&apSocket, 0, sizeof(apSocket));

    unlink(USER_PROTOCOL_STACK_SOCKET);

    if ((sockListen = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        printf("socket error\n");
        return -1;
    }

    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_UNIX;
    strcpy(server_addr.sun_path, USER_PROTOCOL_STACK_SOCKET);

    if (bind(sockListen, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("bind error\n");
        return -1;
    }

    if (listen(sockListen, 5) < 0) {
        printf("listen error\n");
        return -1;
    }

    epollfd = epoll_create(MAX_EVENTS);
    struct epoll_event event;
    event.events = listenEvents;
    event.data.fd = sockListen;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, sockListen, &event) < 0) {
        return -1;
    }

    while(1) {
        int ret = epoll_wait(epollfd, eventList, MAX_EVENTS, -1);

        if (ret < 0) {
            break;
        }

        int n = 0;
        for(n=0; n<ret; n++) {
            if ((eventList[n].events & EPOLLERR) ||
                (eventList[n].events & EPOLLHUP) ||
                !(eventList[n].events & EPOLLIN)) {
              printf("epoll error\n");
              close(eventList[n].data.fd);
              continue;
            }

            if (eventList[n].data.fd == sockListen) {
                AcceptConn(sockListen);

            } else {
                RecvData(eventList[n].data.fd);
            }
        }
    }

    close(epollfd);
    close(sockListen);

    return 0;
}

