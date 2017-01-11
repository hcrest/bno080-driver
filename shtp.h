/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Hillcrest Sensor Hub Transport Protocol (SHTP) API
 */

#ifndef SHTP_H
#define SHTP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
	
// Advertisement TLV tags
#define TAG_NULL 0
#define TAG_GUID 1
#define TAG_MAX_CARGO_PLUS_HEADER_WRITE 2
#define TAG_MAX_CARGO_PLUS_HEADER_READ 3
#define TAG_MAX_TRANSFER_WRITE 4
#define TAG_MAX_TRANSFER_READ 5
#define TAG_NORMAL_CHANNEL 6
#define TAG_WAKE_CHANNEL 7
#define TAG_APP_NAME 8
#define TAG_CHANNEL_NAME 9
#define TAG_ADV_COUNT 10
#define TAG_APP_SPECIFIC 0x80

typedef void shtp_Callback_t(void * cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
typedef void shtp_AdvertCallback_t(void * cookie, uint8_t tag, uint8_t len, uint8_t *value);
typedef void shtp_SendCallback_t(void *cookie);

int shtp_init(void);

int shtp_listenChan(const char * app, const char * chan,
                    shtp_Callback_t *callback, void * cookie);

int shtp_listenAdvert(const char * appName,
                      shtp_AdvertCallback_t *advertCallback, void * cookie);

uint8_t shtp_chanNo(const char * appName, const char * chanName);

int shtp_send(uint8_t channel, uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}    // end of extern "C"
#endif

// #ifdef SHTP_H
#endif
