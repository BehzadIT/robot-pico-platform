#pragma once

/*
 * Minimal lwIP configuration for the Pico 2 W firmware.
 *
 * The service core uses Pico SDK CYW43 background networking and Mongoose's
 * Pico SDK + lwIP socket integration path. Keep BSD socket compatibility
 * enabled here so Mongoose can use the standard socket names it expects.
 */

#define NO_SYS 0
#define LWIP_SOCKET 1
#define LWIP_NETCONN 1
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_DNS 1
#define LWIP_DHCP 1
#define LWIP_IPV4 1
#define LWIP_IPV6 0
#define LWIP_ARP 1
#define LWIP_ETHERNET 1
#define LWIP_RAW 1
#define SYS_LIGHTWEIGHT_PROT 1

#define MEM_ALIGNMENT 4
#define MEM_SIZE (16 * 1024)
#define MEMP_NUM_TCP_SEG 32
#define MEMP_NUM_PBUF 32
#define PBUF_POOL_SIZE 24
#define PBUF_POOL_BUFSIZE 1536

#define TCP_MSS 1460
#define TCP_WND (8 * TCP_MSS)
#define TCP_SND_BUF (8 * TCP_MSS)

#define LWIP_NETIF_HOSTNAME 1
#define LWIP_CHKSUM_ALGORITHM 3

#define LWIP_TIMEVAL_PRIVATE 0
#define LWIP_ERRNO_STDINCLUDE 1
#define LWIP_COMPAT_SOCKETS 2
#define LWIP_POSIX_SOCKETS_IO_NAMES 1
#define MEM_LIBC_MALLOC 0
#define MEMP_MEM_MALLOC 0

#define CYW43_LWIP 1
