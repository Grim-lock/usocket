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

#define RING_SIZE 64
#define RING_FLAGS 0
#define SOCK_NUM 32
#define SOCK_BASE 60000
#define SOCK_PORT(__x) (__x + SOCK_BASE)
#define SOCK_POS(__x) (__x - SOCK_BASE)

static usocket *apSocket[SOCK_NUM] = {};

usocket *usocket_create(int socketFd, uint16_t daddr, uint16_t dport, uint8_t ip_proto) {
    char ring_name[32];

    for (int i = 0; i < SOCK_NUM; i++) {
        if (NULL == apSocket[i]) {
            apSocket[i] = malloc(sizeof(usocket));
            if (NULL != apSocket[i]) {
                bzero(apSocket[i], sizeof(usocket));
                apSocket[i]->socketFd = socketFd;
                apSocket[i]->sport = SOCK_PORT(i);
                apSocket[i]->dport = dport;
                apSocket[i]->daddr = daddr;
                apSocket[i]->ip_proto = ip_proto;
                printf(ring_name, "%hu_to_%hu", apSocket[i]->sport, dport);
            	apSocket[i]->send_ring = rte_ring_create(ring_name, RING_SIZE, rte_socket_id(), RING_FLAGS);
                printf(ring_name, "%hu_to_%hu", dport, apSocket[i]->sport);
            	apSocket[i]->recv_ring = rte_ring_create(ring_name, RING_SIZE, rte_socket_id(), RING_FLAGS);
                apSocket[i]->eventFd = eventfd(0, 0);
            }
            return apSocket[i];
        }
    }

    return NULL;
}

void usocket_delete(int socketFd) {
    char ring_name[32];

    for (int i = 0; i < SOCK_NUM; i++) {
        if ((NULL != apSocket[i]) && (apSocket[i]->socketFd == socketFd)) {
        	rte_ring_free(apSocket[i]->send_ring);
        	rte_ring_free(apSocket[i]->recv_ring);
            close(apSocket[i]->eventFd);
            free(apSocket[i]);
            apSocket[i] = NULL;
        }
    }
}

