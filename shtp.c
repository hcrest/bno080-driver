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
 * Hillcrest Sensor Hub Transport Protocol (SHTP) implementation.
 */

#include <string.h>
#include <stdio.h>

#include "shtp.h"
#include "sh2_hal.h"
#include "sh2_util.h"
#include "sh2_err.h"

#define SH2_MAX_CHANS (8)
#define SH2_MAX_APPS (5)
#define SHTP_APP_NAME_LEN (32)
#define SHTP_CHAN_NAME_LEN (32)

// Defined Globally Unique Identifiers
#define GUID_SHTP (0)

// Command Channel commands and parameters
#define SHTP_CHAN_COMMAND (0)
#define CMD_ADVERTISE (0)
#define CMD_ADVERTISE_SHTP (0)
#define CMD_ADVERTISE_ALL (1)
#define RESP_ADVERTISE (0)

#define SHTP_HDR_LEN (4)

// Note: Payload and transfer length constants do not include header length.
#define SHTP_MAX_PAYLOAD_OUT (SH2_HAL_MAX_TRANSFER - SHTP_HDR_LEN)
#define SHTP_MAX_TRANSFER_OUT (SH2_HAL_MAX_TRANSFER - SHTP_HDR_LEN)
#define INIT_MAX_TRANSFER_OUT (SH2_HAL_MAX_TRANSFER - SHTP_HDR_LEN)

#define SHTP_MAX_PAYLOAD_IN (1200 - SHTP_HDR_LEN)
#define SHTP_MAX_TRANSFER_IN (SH2_HAL_MAX_TRANSFER - SHTP_HDR_LEN)
#define SHTP_INITIAL_READ_LEN (0)

#define TAG_SHTP_VERSION 0x80

// ------------------------------------------------------------------------
// Private Type definitions

typedef struct shtp_App_s {
    uint32_t guid;
    char appName[SHTP_APP_NAME_LEN];
} shtp_App_t;

typedef struct shtp_AppListener_s {
    char appName[SHTP_APP_NAME_LEN];
    shtp_AdvertCallback_t *callback;
    void *cookie;
} shtp_AppListener_t;

typedef struct shtp_Channel_s {
    uint8_t nextOutSeq;
    uint8_t nextInSeq;
    uint32_t guid;  // app id
    char chanName[SHTP_CHAN_NAME_LEN];
    bool wake;
    shtp_Callback_t *callback;
    void *cookie;
} shtp_Channel_t;

typedef struct shtp_ChanListener_s {
    char appName[SHTP_APP_NAME_LEN];
    char chanName[SHTP_CHAN_NAME_LEN];
    shtp_Callback_t *callback;
    void *cookie;
} shtp_ChanListener_t;

#define ADVERT_NEEDED (0)
#define ADVERT_REQUESTED (1)
#define ADVERT_IDLE (2)

typedef struct shtp_s {
    char shtpVersion[8];

    uint8_t advertPhase;

    // stats
    uint32_t tooLargePayloads;
    uint32_t txDiscards;
    uint32_t shortFragments;
    uint32_t badRxChan;
    uint32_t badTxChan;
    
    // transmit support
    uint16_t outMaxPayload;
    uint16_t outMaxTransfer;
    uint8_t outTransfer[SHTP_MAX_TRANSFER_OUT + SHTP_HDR_LEN];

    // receive support
    uint16_t inMaxTransfer;
    uint16_t inRemaining;
    uint8_t  inChan;
    uint8_t  inPayload[SHTP_MAX_PAYLOAD_IN];
    uint16_t inCursor;
    uint32_t inTimestamp;

    // Applications and their listeners
    shtp_App_t app[SH2_MAX_APPS];
    uint8_t    nextApp;
    
    shtp_AppListener_t appListener[SH2_MAX_APPS];
    uint8_t            nextAppListener;

    // Channels and their listeners
    shtp_Channel_t      chan[SH2_MAX_CHANS];
    shtp_ChanListener_t chanListener[SH2_MAX_CHANS];
    uint8_t             nextChanListener;

} shtp_t;

// ------------------------------------------------------------------------
// Forward definitions

static void shtp_onRx(void* cookie, uint8_t* pdata, uint32_t len, uint32_t t_us);
static void addApp(uint32_t guid, const char *appName);
static void addChannel(uint8_t chanNo, uint32_t guid, const char * chanName, bool wake);
static void shtpAdvertHdlr(void *shtp, uint8_t tag, uint8_t len, uint8_t *val);
static void shtpCmdListener(void *shtp, uint8_t *payload, uint16_t len, uint32_t timestamp);
static void addAdvertListener(const char *appName,
                              shtp_AdvertCallback_t *callback, void * cookie);
static int addChanListener(const char * appName, const char * chanName,
                           shtp_Callback_t *callback, void *cookie);
static int toChanNo(const char * appName, const char *chanName);
static int txProcess(uint8_t chan, uint8_t* pData, uint32_t len);

// ------------------------------------------------------------------------
// Private, static data

static shtp_t shtp;

static uint8_t advertise[] = {
    CMD_ADVERTISE,
    CMD_ADVERTISE_ALL
};

// ------------------------------------------------------------------------
// Public API

int shtp_init(void)
{
    // Init stats
    shtp.tooLargePayloads = 0;
    shtp.txDiscards = 0;
    shtp.shortFragments = 0;
    shtp.badRxChan = 0;
    shtp.badTxChan = 0;

    // Init transmit support
    shtp.outMaxPayload = SHTP_MAX_PAYLOAD_OUT;
    shtp.outMaxTransfer = INIT_MAX_TRANSFER_OUT;

    // Init receive support
    shtp.inMaxTransfer = SHTP_MAX_TRANSFER_IN;
    shtp.inRemaining = 0;
    shtp.inCursor = 0;

    // Init SHTP Apps
    for (unsigned int n = 0; n < SH2_MAX_APPS; n++) {
        shtp.app[n].guid = 0xFFFFFFFF;
        strcpy(shtp.app[n].appName, "");
    }
    shtp.nextApp = 0;
    shtp.advertPhase = ADVERT_NEEDED;

    // Init App Listeners
    for (unsigned int n = 0; n < SH2_MAX_APPS; n++) {
        strcpy(shtp.appListener[n].appName, "");
        shtp.appListener[n].callback = 0;
        shtp.appListener[n].cookie = 0;
    }
    shtp.nextAppListener = 0;

    // Init the shtp channels
    for (unsigned int n = 0; n < SH2_MAX_CHANS; n++) {
        shtp.chan[n].nextOutSeq = 0;
        shtp.chan[n].nextInSeq = 0;
        shtp.chan[n].guid = 0xFFFFFFFF;
        strcpy(shtp.chan[n].chanName, "");
        shtp.chan[n].cookie = 0;
        shtp.chan[n].callback = 0;
        shtp.chan[n].wake = false;
    }

    // Init registered channel listeners array
    for (unsigned int n = 0; n < SH2_MAX_CHANS; n++) {
        strcpy(shtp.chanListener[n].appName, "");
        strcpy(shtp.chanListener[n].chanName, "");
        shtp.chanListener[n].cookie = 0;
        shtp.chanListener[n].callback = 0;
    }
    shtp.nextChanListener = 0;

    // Establish SHTP App and command channel a priori.
    addApp(GUID_SHTP, "SHTP");
    addChannel(0, GUID_SHTP, "command", false);

    // Create the control channel for this SHTP instance
    // Register advert listener for SHTP App
    shtp_listenAdvert("SHTP", shtpAdvertHdlr, NULL);
    shtp_listenChan("SHTP", "command", shtpCmdListener, NULL);

    return SH2_OK;
}

void shtp_start(bool dfu)
{
    // Reset device, registering rx callback
    sh2_hal_reset(dfu, shtp_onRx, NULL);
}

// Register a listener for advertisements related to one app
int shtp_listenAdvert(const char * appName, 
                      shtp_AdvertCallback_t *callback, void *cookie)
{
    int rc = SH2_OK;
    
    // Register the advert listener
    addAdvertListener(appName, callback, cookie);

    // Arrange for a new set of advertisements, for this listener
    if (shtp.advertPhase == ADVERT_IDLE) {
        // Request advertisement if one is not already on the way
        rc = shtp_send(SHTP_CHAN_COMMAND, advertise, sizeof(advertise));

        if (rc == SH2_OK) {
            shtp.advertPhase = ADVERT_REQUESTED;
        }
        else {
            shtp.advertPhase = ADVERT_NEEDED;
        }
    }

    return SH2_OK;
}


int shtp_listenChan(const char *app, const char *chan,
                    shtp_Callback_t *callback, void *cookie)
{
    // Balk if app or channel name isn't valid
    if ((app == 0) || (strlen(app) == 0)) return SH2_ERR_BAD_PARAM;
    if ((chan == 0) || (strlen(chan) == 0)) return SH2_ERR_BAD_PARAM;

    return addChanListener(app, chan, callback, cookie);
}

uint8_t shtp_chanNo(const char * appName, const char * chanName)
{
    uint8_t chanNo = 0xFF;
    
    chanNo = toChanNo(appName, chanName);

    return chanNo;
}

int shtp_send(uint8_t chan, uint8_t *payload, uint16_t len)
{
    int ret = SH2_OK;
    
    if (len > shtp.outMaxPayload) {
        return SH2_ERR_BAD_PARAM;
    }
    if (chan >= SH2_MAX_CHANS) {
        shtp.badTxChan++;
        return SH2_ERR_BAD_PARAM;
    }
    
    ret = txProcess(chan, payload, len);

    return ret;
}

// ------------------------------------------------------------------------
// Private methods

static void rxAssemble(uint8_t *in, uint16_t len, uint32_t t_us)
{
    uint16_t payloadLen;
    bool continuation;
    uint8_t chan = 0;
    uint8_t seq = 0;

    // discard invalid short fragments
    if (len < SHTP_HDR_LEN) {
        shtp.shortFragments++;
        return;
    }
    
    // Interpret header fields
    payloadLen = (in[0] + (in[1] << 8)) & (~0x8000);
    continuation = ((in[1] & 0x80) != 0);
    chan = in[2];
    seq = in[3];
    
    if (payloadLen < SHTP_HDR_LEN) {
      shtp.shortFragments++;
      return;
    }

    if ((chan >= SH2_MAX_CHANS) ||
        (chan >= shtp.nextChanListener)) {
        // Invalid channel id.
        shtp.badRxChan++;
        return;
    }
        
    // Discard earlier assembly in progress if the received data doesn't match it.
    if (shtp.inRemaining) {
        // Check this against previously received data.
        if (!continuation ||
            (chan != shtp.inChan) ||
            (seq != shtp.chan[chan].nextInSeq)) {
            // This fragment doesn't fit with previous one, discard earlier data
            shtp.inRemaining = 0;
        }
    }

    if (shtp.inRemaining == 0) {
        // Discard this fragment if it's a continuation of something we don't have.
        if (continuation) {
            return;
        }

        if (payloadLen-SHTP_HDR_LEN > SHTP_MAX_PAYLOAD_IN) {
            // Error: This payload won't fit! Discard it.
            shtp.tooLargePayloads++;
            return;
        }

        // This represents a new payload

        // Store timestamp
        shtp.inTimestamp = t_us;

        // Start a new assembly.
        shtp.inCursor = 0;
        shtp.inChan = chan;
    }

    // Append the new fragment to the payload under construction.
    if (len > payloadLen) {
        // Only use the valid portion of the transfer
        len = payloadLen;
    }
    memcpy(shtp.inPayload + shtp.inCursor, in+SHTP_HDR_LEN, len-SHTP_HDR_LEN);
    shtp.inCursor += len-SHTP_HDR_LEN;
    shtp.inRemaining = payloadLen - len;

    // If whole payload received, deliver it to channel listener.
    if (shtp.inRemaining == 0) {
        
        // Call callback if there is one.
        if (shtp.chan[chan].callback != 0) {
            shtp.chan[chan].callback(shtp.chan[chan].cookie,
                                       shtp.inPayload, shtp.inCursor,
                                       shtp.inTimestamp);
        }
    }

    // Remember next sequence number we expect for this channel.
    shtp.chan[chan].nextInSeq = seq + 1;
}

static void shtp_onRx(void* cookie, uint8_t* pData, uint32_t len, uint32_t t_us)
{
    rxAssemble(pData, len, t_us);
}

// Try to match registered listeners with their channels.
// This is performed every time the underlying Channel, App, Listener data structures are updated.
// As a result, channel number to callback association is fast when receiving packets
static void updateCallbacks(void)
{
    // Figure out which callback is associated with each channel.
    //   Channel -> (GUID, Chan name).
    //   GUID -> App name.
    //   (App name, Chan name) -> Callback

    uint32_t guid;
    const char * appName = 0;
    const char * chanName = 0;
    
    for (int chanNo = 0; chanNo < SH2_MAX_CHANS; chanNo++) {
        // Reset callback for this channel until we find the right one.
        shtp.chan[chanNo].callback = 0;
            
        if (shtp.chan[chanNo].guid == 0xFFFFFFFF) {
            // This channel entry not used.
            continue;
        }

        // Get GUID and Channel Name for this channel
        guid = shtp.chan[chanNo].guid;
        chanName = shtp.chan[chanNo].chanName;

        // Look up App name for this GUID
        appName = 0;
        for (int appNo = 0; appNo < SH2_MAX_APPS; appNo++) {
            if (shtp.app[appNo].guid == guid) {
                appName = shtp.app[appNo].appName;
                break;
            }
        }
        if (appName == 0) {
            // No App registered with this GUID so can't associate channel callback yet.
        }
        else {
            // Look for a listener registered with this app name, channel name
            for (int listenerNo = 0; listenerNo < SH2_MAX_CHANS; listenerNo++) {
                if ((shtp.chanListener[listenerNo].callback != 0) &&
                    (strcmp(appName, shtp.chanListener[listenerNo].appName) == 0) &&
                    (strcmp(chanName, shtp.chanListener[listenerNo].chanName) == 0)) {
                    
                    // This listener is the one for this channel
                    shtp.chan[chanNo].callback = shtp.chanListener[listenerNo].callback;
                    shtp.chan[chanNo].cookie = shtp.chanListener[listenerNo].cookie;
                    break;
                }
            }
        }
    }
}

// Add one to the set of known Apps
static void addApp(uint32_t guid, const char *appName)
{
    shtp_App_t *pApp = 0;

    // Bail out if this GUID is already registered
    for (int n = 0; n < shtp.nextApp; n++) {
        if (shtp.app[n].guid == guid) return;
    }

    // Bail out if no space for more apps
    if (shtp.nextApp >= SH2_MAX_APPS) return;

    // Register this app
    pApp = &shtp.app[shtp.nextApp];
    shtp.nextApp++;
    pApp->guid = guid;
    strcpy(pApp->appName, appName);

    // Re-evaluate channel callbacks
    updateCallbacks();
}

// Add one to the set of known channels
static void addChannel(uint8_t chanNo, uint32_t guid, const char * chanName, bool wake)
{
    if (chanNo >= SH2_MAX_CHANS) return;

    shtp_Channel_t * pChan = &shtp.chan[chanNo];

    // Store channel definition
    pChan->guid = guid;
    strcpy(pChan->chanName, chanName);
    pChan->wake = wake;

    // Init channel-associated data
    pChan->nextOutSeq = 0;
    pChan->nextInSeq = 0;
    pChan->callback = 0;
    pChan->cookie = 0;

    // Re-evaluate channel callbacks
    updateCallbacks();
}


// Callback for SHTP app-specific advertisement tags
static void shtpAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val)
{
    uint16_t x;

    switch (tag) {
        case TAG_MAX_CARGO_PLUS_HEADER_WRITE:
            x = readu16(val) - SHTP_HDR_LEN;
            if (x < SHTP_MAX_PAYLOAD_OUT) {
                shtp.outMaxPayload = x;
            }
            break;
        case TAG_MAX_CARGO_PLUS_HEADER_READ:
            x = readu16(val) - SHTP_HDR_LEN;
            // No need to store this!
            break;
        case TAG_MAX_TRANSFER_WRITE:
            x = readu16(val) - SHTP_HDR_LEN;
            if (x < SHTP_MAX_TRANSFER_OUT) {
                shtp.outMaxTransfer = x;
            } else {
                shtp.outMaxTransfer = SHTP_MAX_TRANSFER_OUT;
            }
            break;
        case TAG_MAX_TRANSFER_READ:
            x = readu16(val) - SHTP_HDR_LEN;
            if (x < SHTP_MAX_TRANSFER_IN) {
                shtp.inMaxTransfer = x;
            }
            break;
        case TAG_SHTP_VERSION:
            if (strlen((const char *)val) < sizeof(shtp.shtpVersion)) {
                strcpy(shtp.shtpVersion, (const char *)val);
            }
            break;
        default:
            break;
    }
}

static void callAdvertHandler(uint32_t guid,
                              uint8_t tag, uint8_t len, uint8_t *val)
{
    // Find app name for this GUID
    const char * appName = 0;
    for (int n = 0; n < SH2_MAX_APPS; n++) {
        if (shtp.app[n].guid == guid) {
            appName = shtp.app[n].appName;
            break;
        }
    }
    if (appName == 0) {
        // Can't associate App name with this GUID
        return;
    }
    
    // Find listener for this app
    for (int n = 0; n < SH2_MAX_APPS; n++)
    {
        if (strcmp(shtp.appListener[n].appName, appName) == 0) {
            // Found matching App entry
            if (shtp.appListener[n].callback != 0) {
                shtp.appListener[n].callback(shtp.appListener[n].cookie, tag, len, val);
                return;
            }
        }
    }
}

static void processAdvertisement(uint8_t *payload, uint16_t payloadLen)
{
    uint8_t tag;
    uint8_t len;
    uint8_t *val;
    uint16_t cursor = 1;
    uint32_t guid = 0;
    char appName[SHTP_APP_NAME_LEN];
    char chanName[SHTP_CHAN_NAME_LEN];
    uint8_t chanNo;
    bool wake;

    strcpy(appName, "");
    strcpy(chanName, "");

    shtp.advertPhase = ADVERT_IDLE;
        
    while (cursor < payloadLen) {
        tag = payload[cursor++];
        len = payload[cursor++];
        val = payload+cursor;
        cursor += len;

        // Process tag
        switch (tag) {
            case TAG_NULL:
                // Reserved value, not a valid tag.
                break;
            case TAG_GUID:
                // A new GUID is being established so terminate advertisement process with earlier app, if any.
                callAdvertHandler(guid, TAG_NULL, 0, 0);
            
                guid = readu32(val);
                strcpy(appName, "");
                strcpy(chanName, "");
                break;
            case TAG_NORMAL_CHANNEL:
                chanNo = readu8(val);
                wake = false;
                break;
            case TAG_WAKE_CHANNEL:
                chanNo = readu8(val);
                wake = true;
                break;
            case TAG_APP_NAME:
                strcpy(appName, (const char *)val);
                addApp(guid, appName);

                // Now that we potentially have a link between current guid and a
                // registered app, start the advertisement process with the app.
                callAdvertHandler(guid, TAG_GUID, 4, (uint8_t *)&guid);
            
                break;
            case TAG_CHANNEL_NAME:
                strcpy(chanName, (const char *)val);
                addChannel(chanNo, guid, (const char *)val, wake);

                // Store channel metadata
                if (chanNo < SH2_MAX_CHANS) {
                    shtp.chan[chanNo].guid = guid;
                    strcpy(shtp.chan[chanNo].chanName, chanName);
                    shtp.chan[chanNo].wake = wake;
                }
                break;
            case TAG_ADV_COUNT:
                // Not yet supported.
                break;
            default:
                // Nothing special needs to be done with this tag.
                break;
        }
        
        // Deliver a TLV entry to the app's handler
        callAdvertHandler(guid, tag, len, val);
    }

    // terminate advertisement process with last app
    callAdvertHandler(guid, TAG_NULL, 0, 0);
}

// Callback for SHTP command channel
static void shtpCmdListener(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    if ((payload == 0) || (len == 0)) return;
    
    uint8_t response = payload[0];

    switch (response) {
        case RESP_ADVERTISE:
            processAdvertisement(payload, len);
            break;
        default:
            // unknown response
            break;
    }
}

// Register a listener for an app (advertisement listener)
static void addAdvertListener(const char *appName,
                              shtp_AdvertCallback_t *callback, void * cookie)
{
    shtp_AppListener_t *pAppListener = 0;

    // Bail out if no space for more apps
    if (shtp.nextAppListener >= SH2_MAX_APPS) return;

    // Register this app
    pAppListener = &shtp.appListener[shtp.nextAppListener];
    shtp.nextAppListener++;
    strcpy(pAppListener->appName, appName);
    pAppListener->callback = callback;
    pAppListener->cookie = cookie;
}

// Register a new channel listener
static int addChanListener(const char * appName, const char * chanName,
                           shtp_Callback_t *callback, void *cookie)
{
    shtp_ChanListener_t *pListener = 0;

    // Bail out if there are too many listeners registered
    if (shtp.nextChanListener >= SH2_MAX_CHANS) return SH2_ERR;

    // Register channel listener
    pListener = &shtp.chanListener[shtp.nextChanListener];
    shtp.nextChanListener++;
    strcpy(pListener->appName, appName);
    strcpy(pListener->chanName, chanName);
    pListener->callback = callback;
    pListener->cookie = cookie;

    // re-evaluate channel callbacks
    updateCallbacks();

    return SH2_OK;
}

static int toChanNo(const char * appName, const char *chanName)
{
    int chan = 0;
    uint32_t guid = 0xFFFFFFFF;

    // Determine GUID for this appname
    for (int n = 0; n < SH2_MAX_APPS; n++) {
        if (strcmp(shtp.app[n].appName, appName) == 0) {
            guid = shtp.app[n].guid;
            break;
        }
    }
    if (guid == 0xFFFFFFFF) return -1;

    for (chan = 0; chan < SH2_MAX_CHANS; chan++) {
        if ((strcmp(shtp.chan[chan].chanName, chanName) == 0) &&
            shtp.chan[chan].guid == guid) {
            // Found match
            return chan;
        }
    }

    // Not found
    return -1;
}

static inline uint16_t min(uint16_t a, uint16_t b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

// Send a cargo as a sequence of transports
static int txProcess(uint8_t chan, uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;
    
    bool continuation = false;
    uint16_t cursor = 0;
    uint16_t remaining = len;

    while (remaining > 0) {
        // determine length of this transfer
        len = min(remaining, shtp.outMaxTransfer);

        // Stage one tranfer in the out buffer
        memcpy(shtp.outTransfer+SHTP_HDR_LEN, pData+cursor, len);
        remaining -= len;
        cursor += len;

        // Add the header len
        len += SHTP_HDR_LEN;

        // Put the header in the out buffer
        shtp.outTransfer[0] = len & 0xFF;
        shtp.outTransfer[1] = (len >> 8) & 0xFF;
        if (continuation) {
            shtp.outTransfer[1] |= 0x80;
        }
        shtp.outTransfer[2] = chan;
        shtp.outTransfer[3] = shtp.chan[chan].nextOutSeq++;

        // Transmit
        int status = sh2_hal_tx(shtp.outTransfer, len);
        if (status != SH2_OK) {
            // Error, throw away this cargo
            shtp.txDiscards++;
            break;
        }

        // For the rest of this transmission, packets are continuations.
        continuation = true;
    }

    return status;
}




