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
 * BNO080 DFU (Download Firmware Update) Implementation.
 */

#include <string.h>
#include <stdio.h>

#include "dfu.h"
#include "sh2_hal.h"
#include "sh2_err.h"

// --- Private Data Types -------------------------------------------------

#define ACK ('s')

#define MAX_PACKET_LEN (64)
#define DFU_MAX_ATTEMPTS (5)

// --- Forward Declarations -----------------------------------------------

static int sendAppSize(unsigned unit, uint32_t appSize);
static int sendPktSize(unsigned unit, uint8_t packetLen);
static int sendPkt(unsigned unit, uint8_t* pData, uint32_t len);

// --- Private Data -------------------------------------------------------

uint8_t dfuBuff[MAX_PACKET_LEN + 2];
uint32_t numRetries;

// --- Public API ---------------------------------------------------------

int dfu(unsigned unit, const HcBin_t *firmware)
{
	int rc;
	int status = SH2_OK;
	uint32_t appLen = 0;
	uint8_t packetLen = 0;
    uint32_t offset = 0;
	const char * s = 0;

	if (firmware == 0) {
		// No firmware object
		return SH2_ERR_BAD_PARAM;
	}

	// Open the hcbin object
	rc = firmware->open();
	if (rc != 0) {
		status = SH2_ERR;
        goto end;
	}

	// Validate firmware matches this implementation
	s = firmware->getMeta("FW-Format");
	if ((s == 0) || (strcmp(s, "BNO_V1") != 0)) {
		// No format info or Incorrect format
		status = SH2_ERR_BAD_PARAM;
        goto close_and_return;
	}

	// Validate firmware is for the right part number
	s = firmware->getMeta("SW-Part-Number");
	if (s == 0) {
	    // No part number info
		status = SH2_ERR_BAD_PARAM;
        goto close_and_return;
	}
	if ((strcmp(s, "1000-3608") != 0) &&
	    (strcmp(s, "1000-3676") != 0)) {
		// Incorrect part number
		status = SH2_ERR_BAD_PARAM;
        goto close_and_return;
	}

    // Validate firmware length
	appLen = firmware->getAppLen();
	if (appLen == 0) {
		// App data is empty
        status = SH2_ERR_BAD_PARAM;
        goto close_and_return;
	}

	// Determine packet length to use
	packetLen = firmware->getPacketLen();
	if ((packetLen == 0) || (packetLen > MAX_PACKET_LEN)) {
		packetLen = MAX_PACKET_LEN;
	}

	// Initiate DFU process

    // Reset part into DFU mode
    status = sh2_hal_reset(unit, true, 0, 0);
    if (status != SH2_OK) {
        goto close_and_return;
    }

    // Send app size
    status = sendAppSize(unit, appLen);
    if (status != SH2_OK) {
        goto close_and_return;
    }

    // Send packet size
    status = sendPktSize(unit, packetLen);
    if (status != SH2_OK) {
        goto close_and_return;
    }
    
    // Send firmware image
    offset = 0;
    while (offset < appLen) {
        uint32_t toSend = appLen - offset;
        if (toSend > packetLen) {
            toSend = packetLen;
        }

        // Extract this packet's content from hcbin
        status = firmware->getAppData(dfuBuff, offset, toSend);
        if (status != SH2_OK) {
            goto close_and_return;
        }
        
        // Send this packet's contents
        status = sendPkt(unit, dfuBuff, toSend);
        if (status != SH2_OK) {
            goto close_and_return;
        }

        // update loop variables
        offset += toSend;
    }

close_and_return:
    // close firmware
    firmware->close();

end:
	return status;
}

// --- Private utility functions --------------------------------------------------------------


static void write32be(uint8_t *buf, uint32_t value)
{
	*buf++ = (value >> 24) & 0xFF;
	*buf++ = (value >> 16) & 0xFF;
	*buf++ = (value >> 8) & 0xFF;
	*buf++ = (value >> 0) & 0xFF;
}

static void appendCrc(uint8_t *packet, uint8_t len)
{
  uint16_t crc;
  uint16_t x;

  // compute CRC of packet
  crc = 0xFFFF;
  for (int n = 0; n < len; n++) {
    x = (uint16_t)(packet[n]) << 8;
    for (int i = 0; i < 8; i++) {
      if ((crc ^ x) & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      }
      else {
        crc = crc << 1;
      }
      x <<= 1;
    }
  }

  // Append the CRC to packet
  packet[len] = (crc >> 8) & 0xFF;
  packet[len+1] = crc & 0xFF;
}

// I/O Utility functions
static int dfuSend(unsigned unit, uint8_t* pData, uint32_t len)
{
    unsigned retries = 0;
    int status = SH2_OK;
    uint8_t ack;

    while (retries < DFU_MAX_ATTEMPTS) {
        // Do write
        status = sh2_hal_tx(unit, pData, len);
        if (status == SH2_OK) {
            // Read ack
            status = sh2_hal_rx(unit, &ack, 1);
            if (status == SH2_OK) {
                if (ack == ACK) {
                    // Everything went fine, break out of retry cycle
                    break;
                }
                else {
                    status = SH2_ERR;
                }
            }
        }

        // Problem: try again.
        retries++;
        numRetries++;
    }

    return status;
}

static int sendAppSize(unsigned unit, uint32_t appSize)
{
	write32be(dfuBuff, appSize);
	appendCrc(dfuBuff, 4);

    return dfuSend(unit, dfuBuff, 6);
}

static int sendPktSize(unsigned unit, uint8_t packetLen)
{
    dfuBuff[0] = packetLen;
	appendCrc(dfuBuff, 1);
    
    return dfuSend(unit, dfuBuff, 3);
}

static int sendPkt(unsigned unit, uint8_t* pData, uint32_t len)
{
    memcpy(dfuBuff, pData, len);
    appendCrc(dfuBuff, len);
    
    return dfuSend(unit, dfuBuff, len+2);  // plus 2 for CRC
}

