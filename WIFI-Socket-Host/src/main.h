/**
 * \file
 *
 * \brief MAIN configuration.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID                    "OPEN" //"TP-LINK_9AE8" /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                     "12345678" /**< Password for Destination SSID */
#define MAIN_WIFI_M2M_PRODUCT_NAME        "SAME70"
#define MAIN_WIFI_M2M_SERVER_IP           0xFFFFFFFF /* 255.255.255.255 */


//#define MAIN_PREFIX_BUFFER                "GET /led=status HTTP/1.1\r\nHost: api.openweathermap.org\r\nAccept: */*\r\n\r\n"
#define MAIN_PREFIX_BUFFER                  "GET /led_status"
#define MAIN_POST                           " HTTP/1.1\r\n Accept: */*\r\n\r\n"

#define MAIN_WIFI_M2M_BUFFER_SIZE          1460

#define MAIN_SERVER_PORT                   (5000)
#define MAIN_SERVER_IP                     0x9000A8C0//0x6601A8C0//0x9000A8C0 // 0x8A00A8C0  // 192.168.0.138 192.168.0.144

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
