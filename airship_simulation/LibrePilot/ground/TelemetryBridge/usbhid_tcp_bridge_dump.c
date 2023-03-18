/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */
/**
 * This is a small test program that connects to OpenPilot/CC via USB and writes a .opl compatible stream to stdout
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             Max Planck Institute for intelligent systems, http://www.is.mpg.de Copyright (C) 2016.
 */

#include <usb.h>
// #include <libudev.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

// recveive - receive a packet
// Inputs:
// buf = buffer containing packet to send
// len = number of bytes to transmit
// timeout = time to wait, in milliseconds
// Output:
// number of bytes received, or -1 on error
//


#define BUF_SIZE 16384
unsigned char tcp2usbBUF[BUF_SIZE];
unsigned char usb2tcpBUF[BUF_SIZE];
volatile int tcp2usbFILL;
volatile int usb2tcpFILL;

pthread_mutex_t tcp2usbMUTEX, usb2tcpMUTEX;
pthread_cond_t tcp2usbSEM, usb2tcpSEM;

int ep_in = 0, ep_out = 0;
usb_dev_handle *dev_handle, *OpenPilot = NULL;


int write_buf(unsigned char *src, int size, unsigned char *buffer, volatile int *fill)
{
    if (*fill + size > BUF_SIZE) {
        return 0; // bufer overflow
    }
    memcpy(buffer, src, size);
    *fill += size;
    return size;
}

int read_buf(unsigned char *dst, unsigned char *buffer, volatile int *fill)
{
    int size = *fill;

    memcpy(dst, buffer, size);
    *fill = 0;
    return size;
}

int opreceive(usb_dev_handle *device, int endpoint, void *buf, int timeout)
{
    int8_t tmpBuffer[64];

    if (!buf) {
        return -1;
    }
    if (!device) {
        return -1;
    }
    if (!endpoint) {
        return -1;
    }
    int received = 0;


    received = usb_interrupt_read(device, endpoint, tmpBuffer, 64, timeout);
    if (received > 0) {
        received = tmpBuffer[1];
        if (received > 62) {
            received = 62;
        }
        memcpy(buf, &tmpBuffer[2], received);
        return received;
    }
    return received;
}

int oprealsend(usb_dev_handle *device, int endpoint, void *buf, int size, int timeout)
{
    int8_t tmpBuffer[64];

    if (!buf) {
        return -1;
    }
    if (!device) {
        return -1;
    }
    if (!endpoint) {
        return -1;
    }
    int sent = 0;
    tmpBuffer[0] = 2;
    tmpBuffer[1] = (unsigned char)(size & 0xff);
    memcpy(&tmpBuffer[2], buf, size);

    if (size > 0) {
        sent = usb_interrupt_write(device, endpoint, tmpBuffer, 64, timeout);
    }
    return size;
}

int opsend(usb_dev_handle *device, int endpoint, void *buf, int size, int timeout)
{
    while (size > 62) {
        if (oprealsend(device, endpoint, buf, 62, timeout) < 0) {
            return -1;
        }
        size -= 62;
        buf  += 62;
    }
    return oprealsend(device, endpoint, buf, size, timeout);
}

int tcpsend(int client_sock, void *buf, int size)
{
    while (size > 62) {
        if (send(client_sock, buf, 62, 0) < 0) {
            return -1;
        }
        size -= 62;
        buf  += 62;
    }
    return send(client_sock, buf, size, 0);
}

int64_t timeDifference(struct timeval *old, struct timeval *new)
{
    time_t seconds;
    int64_t mseconds;

    mseconds = (new->tv_usec - old->tv_usec) / 1000;
    mseconds = mseconds + (new->tv_sec - old->tv_sec) * 1000;

    return mseconds;
}


int socket_desc;

volatile int client_sock = 0;

void *listener(void *unused)
{
    int c;
    struct sockaddr_in client;

    // Accept and incoming connection
    fprintf(stderr, "Waiting for incoming connections...\n");
    c = sizeof(struct sockaddr_in);

    while (client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t *)&c)) {
        fprintf(stderr, "Connection accepted\n");

        char buffer[62];
        int n;
        while ((n = recv(client_sock, buffer, 62, 0)) > 0) {
            pthread_mutex_lock(&tcp2usbMUTEX);
            write_buf(buffer, n, tcp2usbBUF, &tcp2usbFILL);
            pthread_cond_broadcast(&tcp2usbSEM);
            pthread_mutex_unlock(&tcp2usbMUTEX);
            fprintf(stderr, "+");
        }
        close(client_sock);
        client_sock = 0;
    }

    if (client_sock < 0) {
        perror("accept failed\n");
        return 0;
    }
    return 0;
}
void *tcpwriter(void *unused)
{
    pthread_mutex_lock(&usb2tcpMUTEX);
    while (pthread_cond_wait(&usb2tcpSEM, &usb2tcpMUTEX) == 0) {
        if (client_sock) {
            unsigned char buffer[BUF_SIZE];
            int len = read_buf(buffer, usb2tcpBUF, &usb2tcpFILL);
            if (tcpsend(client_sock, buffer, len) < 0) {
                fprintf(stderr, "FAIL!!!!! TCP Write Error!!!\n");
            }
        }
    }
    fprintf(stderr, "FAIL!!!!!!! EXITUS\n");
}
void *usbwriter(void *unused)
{
    pthread_mutex_lock(&tcp2usbMUTEX);
    while (pthread_cond_wait(&tcp2usbSEM, &tcp2usbMUTEX) == 0) {
        unsigned char buffer[BUF_SIZE];
        int len = read_buf(buffer, tcp2usbBUF, &tcp2usbFILL);
        if (opsend(OpenPilot, ep_out, buffer, len, 1000) < 0) {
            fprintf(stderr, "FAIL!!!!! TCP Write Error!!!\n");
        }
    }
    fprintf(stderr, "FAIL!!!!!!! EXITUS\n");
}

int main()
{
    int vendor     = 0x20a0;
    int product_op = 0x415e;
    int product_cc = 0x415b;


    struct usb_bus *bus;
    struct usb_device *dev;
    struct usb_interface *iface;
    struct usb_interface_descriptor *desc;
    struct usb_endpoint_descriptor *ep;
    unsigned char buf[1024];

    int claimed;
    int started = 0;
    struct timeval starttime, ctime;

    struct sockaddr_in server;
    pthread_t listener_thread, tcpwriter_thread, usbwriter_thread;

    pthread_mutex_init(&tcp2usbMUTEX, NULL);
    pthread_mutex_init(&usb2tcpMUTEX, NULL);
    pthread_cond_init(&tcp2usbSEM, NULL);
    pthread_cond_init(&usb2tcpSEM, NULL);


    // Create socket
    socket_desc = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_desc == -1) {
        fprintf(stderr, "Could not create socket\n");
        return 1;
    }
    fprintf(stderr, "Socket created\n");
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port   = htons(9000);
    // Bind
    if (bind(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0) {
        // print the error message
        perror("bind failed. Error");
        return 1;
    }
    fprintf(stderr, "bind done\n");

    // Listen
    listen(socket_desc, 3);
    if (pthread_create(&tcpwriter_thread, NULL, tcpwriter, NULL)) {
        perror("could not create thread\n");
        return 1;
    }
    if (pthread_create(&usbwriter_thread, NULL, usbwriter, NULL)) {
        perror("could not create thread\n");
        return 1;
    }
    if (pthread_create(&listener_thread, NULL, listener, NULL)) {
        perror("could not create thread\n");
        return 1;
    }


    while (1) {
        usb_set_debug(99);
        usb_init();
        usb_find_busses();
        usb_find_devices();

        for (bus = usb_get_busses(); bus; bus = bus->next) {
            fprintf(stderr, "usbbus found\n");
            for (dev = bus->devices; dev; dev = dev->next) {
                fprintf(stderr, "usbdevice found - %04X:%04X\n", dev->descriptor.idVendor, dev->descriptor.idProduct);
                if (dev->descriptor.idVendor != vendor) {
                    continue;
                }
                if (dev->descriptor.idProduct != product_op && dev->descriptor.idProduct != product_cc) {
                    continue;
                }
                if (!dev->config) {
                    continue;
                }
                if (dev->config->bNumInterfaces < 1) {
                    continue;
                }
                fprintf(stderr, "device: vid=%04X, pic=%04X, with %d iface",
                        dev->descriptor.idVendor,
                        dev->descriptor.idProduct,
                        dev->config->bNumInterfaces);
                iface      = dev->config->interface;
                dev_handle = NULL;
                claimed    = 0;
                int i;
                for (i = 0; i < dev->config->bNumInterfaces && iface; i++, iface++) {
                    desc = iface->altsetting;
                    if (!desc) {
                        continue;
                    }

                    fprintf(stderr, "  type %d, %d, %d", desc->bInterfaceClass, desc->bInterfaceSubClass, desc->bInterfaceProtocol);

                    if (desc->bInterfaceClass != 3) {
                        continue;
                    }
                    if (desc->bInterfaceSubClass != 0) {
                        continue;
                    }
                    if (desc->bInterfaceProtocol != 0) {
                        continue;
                    }

                    ep    = desc->endpoint;
                    ep_in = ep_out = 0;
                    int n;
                    for (n = 0; n < desc->bNumEndpoints; n++, ep++) {
                        if (ep->bEndpointAddress & 0x80) {
                            if (!ep_in) {
                                ep_in = ep->bEndpointAddress & 0x7F;
                            }
                            fprintf(stderr, "    IN endpoint %X\n", ep_in);
                        } else {
                            if (!ep_out) {
                                ep_out = ep->bEndpointAddress;
                            }
                            fprintf(stderr, "    OUT endpoint %X\n", ep_out);
                        }
                    }
                    if (!ep_in) {
                        continue;
                    }

                    if (!dev_handle) {
                        dev_handle = usb_open(dev);
                        if (!dev_handle) {
                            fprintf(stderr, "  unable to open device\n");
                            break;
                        }
                    }
                    fprintf(stderr, "  hid interface (generic)\n");
                    if (usb_get_driver_np(dev_handle, i, (char *)buf, sizeof(buf)) >= 0) {
                        fprintf(stderr, "  in use by driver \"%s\"", buf);
                        if (usb_detach_kernel_driver_np(dev_handle, i) < 0) {
                            fprintf(stderr, "  unable to detach from kernel");
                            continue;
                        }
                    }

                    if (usb_claim_interface(dev_handle, i) < 0) {
                        fprintf(stderr, "  unable claim interface %d", i);
                        continue;
                    }

                    int len;
                    len = usb_control_msg(dev_handle, 0x81, 6, 0x2200, i, (char *)buf, sizeof(buf), 250);
                    fprintf(stderr, "  descriptor, len=%d\n", len);
                    if (len < 2) {
                        usb_release_interface(dev_handle, i);
                        continue;
                    }

                    /*hid = (struct hid_struct *)malloc(sizeof(struct hid_struct));
                       if (!hid)
                       {
                            usb_release_interface(dev_handle, i);
                            continue;
                       }*/

                    fprintf(stderr, "found :)))) \n");
                    /*
                       hid->usb = dev_handle;
                       hid->iface = i;
                       hid->ep_in = ep_in;
                       hid->ep_out = ep_out;
                       hid->open = 1;
                       add_hid(hid);*/

                    claimed++;
                    // count++;
                    // if (count >= max) return count;
                    OpenPilot = dev_handle;
                }

                if (dev_handle && !claimed) {
                    usb_close(dev_handle);
                }
            }
        }
        if (!OpenPilot) {
            fprintf(stderr, "no such device\n");
            // return -1;
        } else {
            char buffer[64];
            int n = 0;
            if (!started) {
                started = 1;
                gettimeofday(&starttime, NULL);
            }
            while (1) {
                do {
                    n = opreceive(OpenPilot, ep_in, buffer, 1000);
                } while (n == -ETIMEDOUT);
                if (n <= 0) {
                    break;
                }
                gettimeofday(&ctime, NULL);
                pthread_mutex_lock(&usb2tcpMUTEX);
                write_buf(buffer, n, usb2tcpBUF, &usb2tcpFILL);
                pthread_cond_broadcast(&usb2tcpSEM);
                pthread_mutex_unlock(&usb2tcpMUTEX);
                uint32_t timestamp = timeDifference(&starttime, &ctime);
                uint64_t dataSize  = n;

                fwrite((uint8_t *)&timestamp, sizeof(timestamp), 1, stdout);
                fwrite((uint8_t *)&dataSize, sizeof(dataSize), 1, stdout);
                fwrite(buffer, 1, dataSize, stdout);
                // fprintf(stderr, " %i: %i\n", timestamp, (uint32_t)dataSize);
                fprintf(stderr, ".");
            }
            fprintf(stderr, "aborting: %s\n", strerror(-n));
        }
        sleep(1);
    }
}
