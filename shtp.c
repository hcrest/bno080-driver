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
    unsigned unit;
    char shtpVersion[8];

    uint8_t advertPhase;

    // stats
    uint32_t tooLargePayloads;
    uint32_t txDiscards;
    uint32_t shortFragments;
    
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
static void addApp(shtp_t *pShtp, uint32_t guid, const char *appName);
static void addChannel(shtp_t *pShtp, uint8_t chanNo, uint32_t guid, const char * chanName, bool wake);
static void shtpAdvertHdlr(void *shtp, uint8_t tag, uint8_t len, uint8_t *val);
static void shtpCmdListener(void *shtp, uint8_t *payload, uint16_t len, uint32_t timestamp);
static void addAdvertListener(shtp_t *pShtp, const char *appName,
                              shtp_AdvertCallback_t *callback, void * cookie);
static int addChanListener(shtp_t *pShtp, const char * appName, const char * chanName,
                           shtp_Callback_t *callback, void *cookie);
static int toChanNo(shtp_t *pShtp, const char * appName, const char *chanName);
static int txProcess(shtp_t* pShtp, uint8_t chan, uint8_t* pData, uint32_t len);

// ------------------------------------------------------------------------
// Private, static data

static shtp_t shtp[SH2_UNITS];

static uint8_t advertise[] = {
    CMD_ADVERTISE,
    CMD_ADVERTISE_ALL
};

static uint32_t onRxBadUnit = 0;

// ------------------------------------------------------------------------
// Public API

int shtp_init(unsigned unit)
{
    shtp_t *pShtp = &shtp[unit];
    if (unit >= ARRAY_LEN(shtp)) {
        return SH2_ERR_BAD_PARAM;
    }

    pShtp->unit = unit;

    // Reset device for SH2 operation, registering rx callback
    sh2_hal_reset(unit, false, shtp_onRx, (void*)unit);

    // Init stats
    pShtp->tooLargePayloads = 0;
    pShtp->txDiscards = 0;
    pShtp->shortFragments = 0;

    // Init transmit support
    pShtp->outMaxPayload = SHTP_MAX_PAYLOAD_OUT;
    pShtp->outMaxTransfer = INIT_MAX_TRANSFER_OUT;

    // Init receive support
    pShtp->inMaxTransfer = SHTP_MAX_TRANSFER_IN;
    pShtp->inRemaining = 0;
    pShtp->inCursor = 0;

    // Init SHTP Apps
    for (unsigned n = 0; n < SH2_MAX_APPS; n++) {
        pShtp->app[n].guid = 0xFFFFFFFF;
        strcpy(pShtp->app[n].appName, "");
    }
    pShtp->nextApp = 0;
    pShtp->advertPhase = ADVERT_NEEDED;

    // Init App Listeners
    for (unsigned n = 0; n < SH2_MAX_APPS; n++) {
        strcpy(pShtp->appListener[n].appName, "");
        pShtp->appListener[n].callback = 0;
        pShtp->appListener[n].cookie = 0;
    }
    pShtp->nextAppListener = 0;

    // Init the shtp channels
    for (unsigned n = 0; n < SH2_MAX_CHANS; n++) {
        pShtp->chan[n].nextOutSeq = 0;
        pShtp->chan[n].nextInSeq = 0;
        pShtp->chan[n].guid = 0xFFFFFFFF;
        strcpy(pShtp->chan[n].chanName, "");
        pShtp->chan[n].cookie = 0;
        pShtp->chan[n].callback = 0;
        pShtp->chan[n].wake = false;
    }

    // Init registered channel listeners array
    for (unsigned n = 0; n < SH2_MAX_CHANS; n++) {
        strcpy(pShtp->chanListener[n].appName, "");
        strcpy(pShtp->chanListener[n].chanName, "");
        pShtp->chanListener[n].cookie = 0;
        pShtp->chanListener[n].callback = 0;
    }
    pShtp->nextChanListener = 0;

    // Establish SHTP App and command channel a priori.
    addApp(pShtp, GUID_SHTP, "SHTP");
    addChannel(pShtp, 0, GUID_SHTP, "command", false);

    // Create the control channel for this SHTP instance
    // Register advert listener for SHTP App
    shtp_listenAdvert(unit, "SHTP", shtpAdvertHdlr, (void *)unit);
    shtp_listenChan(unit, "SHTP", "command", shtpCmdListener, (void *)unit);

    return SH2_OK;
}

// Register a listener for advertisements related to one app
int shtp_listenAdvert(unsigned unit, const char * appName, 
                      shtp_AdvertCallback_t *callback, void *cookie)
{
    int rc = SH2_OK;
    
    shtp_t *pShtp = &shtp[unit];
    if (unit >= ARRAY_LEN(shtp)) {
        return SH2_ERR_BAD_PARAM;
    }

    // Register the advert listener
    addAdvertListener(pShtp, appName, callback, cookie);

    // Arrange for a new set of advertisements, for this listener
    if (pShtp->advertPhase == ADVERT_IDLE) {
        // Request advertisement if one is not already on the way
        rc = shtp_send(unit, SHTP_CHAN_COMMAND, advertise, sizeof(advertise));

        if (rc == SH2_OK) {
            pShtp->advertPhase = ADVERT_REQUESTED;
        }
        else {
            pShtp->advertPhase = ADVERT_NEEDED;
        }
    }

    return SH2_OK;
}


int shtp_listenChan(unsigned unit, const char *app, const char *chan,
                    shtp_Callback_t *callback, void *cookie)
{
    shtp_t *pShtp = &shtp[unit];
    if (unit >= ARRAY_LEN(shtp)) {
        return SH2_ERR_BAD_PARAM;
    }

    // Balk if app or channel name isn't valid
    if ((app == 0) || (strlen(app) == 0)) return SH2_ERR_BAD_PARAM;
    if ((chan == 0) || (strlen(chan) == 0)) return SH2_ERR_BAD_PARAM;

    return addChanListener(pShtp, app, chan, callback, cookie);
}

uint8_t shtp_chanNo(unsigned unit, const char * appName, const char * chanName)
{
    uint8_t chanNo = 0xFF;
    
    shtp_t *pShtp = &shtp[unit];
    if (unit >= ARRAY_LEN(shtp)) {
        return 0xFF;
    }
    
    chanNo = toChanNo(pShtp, appName, chanName);

    return chanNo;
}

int shtp_send(unsigned unit, uint8_t chan, uint8_t *payload, uint16_t len)
{
    int ret = SH2_OK;
    shtp_t *pShtp = &shtp[unit];
    if (unit >= ARRAY_LEN(shtp)) {
        return SH2_ERR_BAD_PARAM;
    }
        
    
    if (len > pShtp->outMaxPayload) {
        return SH2_ERR_BAD_PARAM;
    }
    
    ret = txProcess(pShtp, chan, payload, len);

    return ret;
}

// ------------------------------------------------------------------------
// Private methods

static void rxAssemble(shtp_t *pShtp, uint8_t *in, uint16_t len, uint32_t t_us)
{
    uint16_t payloadLen;
    bool continuation;
    uint8_t chan = 0;
    uint8_t seq = 0;

    // discard invalid short fragments
    if (len < SHTP_HDR_LEN) {
        pShtp->shortFragments++;
        return;
    }
    
    // Interpret header fields
    payloadLen = (in[0] + (in[1] << 8)) & (~0x8000);
    continuation = ((in[1] & 0x80) != 0);
    chan = in[2];
    seq = in[3];
    
    if (payloadLen < SHTP_HDR_LEN) {
      pShtp->shortFragments++;
      return;
    }
        
    // Discard earlier assembly in progress if the received data doesn't match it.
    if (pShtp->inRemaining) {
        // Check this against previously received data.
        if (!continuation ||
            (chan != pShtp->inChan) ||
            (seq != pShtp->chan[chan].nextInSeq)) {
            // This fragment doesn't fit with previous one, discard earlier data
            pShtp->inRemaining = 0;
        }
    }

    if (pShtp->inRemaining == 0) {
        // Discard this fragment if it's a continuation of something we don't have.
        if (continuation) {
            return;
        }

        if (payloadLen-SHTP_HDR_LEN > SHTP_MAX_PAYLOAD_IN) {
            // Error: This payload won't fit! Discard it.
            pShtp->tooLargePayloads++;
            return;
        }

        // This represents a new payload

        // Store timestamp
        pShtp->inTimestamp = t_us;

        // Start a new assembly.
        pShtp->inCursor = 0;
        pShtp->inChan = chan;
    }

    // Append the new fragment to the payload under construction.
    if (len > payloadLen) {
        // Only use the valid portion of the transfer
        len = payloadLen;
    }
    memcpy(pShtp->inPayload + pShtp->inCursor, in+SHTP_HDR_LEN, len-SHTP_HDR_LEN);
    pShtp->inCursor += len-SHTP_HDR_LEN;
    pShtp->inRemaining = payloadLen - len;

    // If whole payload received, deliver it to channel listener.
    if (pShtp->inRemaining == 0) {
        
        // Call callback if there is one.
        if (pShtp->chan[chan].callback != 0) {
            pShtp->chan[chan].callback(pShtp->chan[chan].cookie,
                                       pShtp->inPayload, pShtp->inCursor,
                                       pShtp->inTimestamp);
        }
    }

    // Remember next sequence number we expect for this channel.
    pShtp->chan[chan].nextInSeq = seq + 1;
}

static void shtp_onRx(void* cookie, uint8_t* pData, uint32_t len, uint32_t t_us)
{
    unsigned unit = (unsigned)cookie;
    shtp_t* pShtp = &shtp[unit];

    if (unit >= ARRAY_LEN(shtp)) {
        onRxBadUnit++;
        return;
    }

    rxAssemble(pShtp, pData, len, t_us);
}

// Try to match registered listeners with their channels.
// This is performed every time the underlying Channel, App, Listener data structures are updated.
// As a result, channel number to callback association is fast when receiving packets
static void updateCallbacks(shtp_t *pShtp)
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
        pShtp->chan[chanNo].callback = 0;
            
        if (pShtp->chan[chanNo].guid == 0xFFFFFFFF) {
            // This channel entry not used.
            continue;
        }

        // Get GUID and Channel Name for this channel
        guid = pShtp->chan[chanNo].guid;
        chanName = pShtp->chan[chanNo].chanName;

        // Look up App name for this GUID
        appName = 0;
        for (int appNo = 0; appNo < SH2_MAX_APPS; appNo++) {
            if (pShtp->app[appNo].guid == guid) {
                appName = pShtp->app[appNo].appName;
                break;
            }
        }
        if (appName == 0) {
            // No App registered with this GUID so can't associate channel callback yet.
        }
        else {
            // Look for a listener registered with this app name, channel name
            for (int listenerNo = 0; listenerNo < SH2_MAX_CHANS; listenerNo++) {
                if ((pShtp->chanListener[listenerNo].callback != 0) &&
                    (strcmp(appName, pShtp->chanListener[listenerNo].appName) == 0) &&
                    (strcmp(chanName, pShtp->chanListener[listenerNo].chanName) == 0)) {
                    
                    // This listener is the one for this channel
                    pShtp->chan[chanNo].callback = pShtp->chanListener[listenerNo].callback;
                    pShtp->chan[chanNo].cookie = pShtp->chanListener[listenerNo].cookie;
                    break;
                }
            }
        }
    }
}

// Add one to the set of known Apps
static void addApp(shtp_t *pShtp, uint32_t guid, const char *appName)
{
    shtp_App_t *pApp = 0;

    // Bail out if this GUID is already registered
    for (int n = 0; n < pShtp->nextApp; n++) {
        if (pShtp->app[n].guid == guid) return;
    }

    // Bail out if no space for more apps
    if (pShtp->nextApp >= SH2_MAX_APPS) return;

    // Register this app
    pApp = &pShtp->app[pShtp->nextApp];
    pShtp->nextApp++;
    pApp->guid = guid;
    strcpy(pApp->appName, appName);

    // Re-evaluate channel callbacks
    updateCallbacks(pShtp);
}

// Add one to the set of known channels
static void addChannel(shtp_t *pShtp, uint8_t chanNo, uint32_t guid, const char * chanName, bool wake)
{
    if (chanNo >= SH2_MAX_CHANS) return;

    shtp_Channel_t * pChan = &pShtp->chan[chanNo];

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
    updateCallbacks(pShtp);
}


// Callback for SHTP app-specific advertisement tags
static void shtpAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val)
{
    unsigned unit = (unsigned)cookie;
    shtp_t *pShtp = &shtp[unit];
    uint16_t x;

    switch (tag) {
        case TAG_MAX_CARGO_PLUS_HEADER_WRITE:
            x = readu16(val) - SHTP_HDR_LEN;
            if (x < SHTP_MAX_PAYLOAD_OUT) {
                pShtp->outMaxPayload = x;
            }
            break;
        case TAG_MAX_CARGO_PLUS_HEADER_READ:
            x = readu16(val) - SHTP_HDR_LEN;
            // No need to store this!
            break;
        case TAG_MAX_TRANSFER_WRITE:
            x = readu16(val) - SHTP_HDR_LEN;
            if (x < SHTP_MAX_TRANSFER_OUT) {
                pShtp->outMaxTransfer = x;
            } else {
                pShtp->outMaxTransfer = SHTP_MAX_TRANSFER_OUT;
            }
            break;
        case TAG_MAX_TRANSFER_READ:
            x = readu16(val) - SHTP_HDR_LEN;
            if (x < SHTP_MAX_TRANSFER_IN) {
                pShtp->inMaxTransfer = x;
            }
            break;
        case TAG_SHTP_VERSION:
            if (strlen((const char *)val) < sizeof(pShtp->shtpVersion)) {
                strcpy(pShtp->shtpVersion, (const char *)val);
            }
            break;
        default:
            break;
    }
}

static void callAdvertHandler(shtp_t *pShtp, uint32_t guid,
                              uint8_t tag, uint8_t len, uint8_t *val)
{
    // Find app name for this GUID
    const char * appName = 0;
    for (int n = 0; n < SH2_MAX_APPS; n++) {
        if (pShtp->app[n].guid == guid) {
            appName = pShtp->app[n].appName;
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
        if (strcmp(pShtp->appListener[n].appName, appName) == 0) {
            // Found matching App entry
            if (pShtp->appListener[n].callback != 0) {
                pShtp->appListener[n].callback(pShtp->appListener[n].cookie, tag, len, val);
                return;
            }
        }
    }
}

static void processAdvertisement(shtp_t *pShtp, uint8_t *payload, uint16_t payloadLen)
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

    pShtp->advertPhase = ADVERT_IDLE;
        
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
                callAdvertHandler(pShtp, guid, TAG_NULL, 0, 0);
            
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
                addApp(pShtp, guid, appName);

                // Now that we potentially have a link between current guid and a
                // registered app, start the advertisement process with the app.
                callAdvertHandler(pShtp, guid, TAG_GUID, 4, (uint8_t *)&guid);
            
                break;
            case TAG_CHANNEL_NAME:
                strcpy(chanName, (const char *)val);
                addChannel(pShtp, chanNo, guid, (const char *)val, wake);

                // Store channel metadata
                if (chanNo < SH2_MAX_CHANS) {
                    pShtp->chan[chanNo].guid = guid;
                    strcpy(pShtp->chan[chanNo].chanName, chanName);
                    pShtp->chan[chanNo].wake = wake;
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
        callAdvertHandler(pShtp, guid, tag, len, val);
    }

    // terminate advertisement process with last app
    callAdvertHandler(pShtp, guid, TAG_NULL, 0, 0);
}

// Callback for SHTP command channel
static void shtpCmdListener(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    unsigned unit = (unsigned)cookie;
    shtp_t *pShtp = &shtp[unit];
    
    if ((payload == 0) || (len == 0)) return;
    
    uint8_t response = payload[0];

    switch (response) {
        case RESP_ADVERTISE:
            processAdvertisement(pShtp, payload, len);
            break;
        default:
            // unknown response
            break;
    }
}

// Register a listener for an app (advertisement listener)
static void addAdvertListener(shtp_t *pShtp, const char *appName,
                              shtp_AdvertCallback_t *callback, void * cookie)
{
    shtp_AppListener_t *pAppListener = 0;

    // Bail out if no space for more apps
    if (pShtp->nextAppListener >= SH2_MAX_APPS) return;

    // Register this app
    pAppListener = &pShtp->appListener[pShtp->nextAppListener];
    pShtp->nextAppListener++;
    strcpy(pAppListener->appName, appName);
    pAppListener->callback = callback;
    pAppListener->cookie = cookie;
}

// Register a new channel listener
static int addChanListener(shtp_t *pShtp, const char * appName, const char * chanName,
                           shtp_Callback_t *callback, void *cookie)
{
    shtp_ChanListener_t *pListener = 0;

    // Bail out if there are too many listeners registered
    if (pShtp->nextChanListener >= SH2_MAX_CHANS) return SH2_ERR;

    // Register channel listener
    pListener = &pShtp->chanListener[pShtp->nextChanListener];
    pShtp->nextChanListener++;
    strcpy(pListener->appName, appName);
    strcpy(pListener->chanName, chanName);
    pListener->callback = callback;
    pListener->cookie = cookie;

    // re-evaluate channel callbacks
    updateCallbacks(pShtp);

    return SH2_OK;
}

static int toChanNo(shtp_t *pShtp, const char * appName, const char *chanName)
{
    int chan = 0;
    uint32_t guid = 0xFFFFFFFF;

    // Determine GUID for this appname
    for (int n = 0; n < SH2_MAX_APPS; n++) {
        if (strcmp(pShtp->app[n].appName, appName) == 0) {
            guid = pShtp->app[n].guid;
            break;
        }
    }
    if (guid == 0xFFFFFFFF) return -1;

    for (chan = 0; chan < SH2_MAX_CHANS; chan++) {
        if ((strcmp(pShtp->chan[chan].chanName, chanName) == 0) &&
            pShtp->chan[chan].guid == guid) {
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
static int txProcess(shtp_t* pShtp, uint8_t chan, uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;
    
    bool continuation = false;
    uint16_t cursor = 0;
    uint16_t remaining = len;

    while (remaining > 0) {
        // determine length of this transfer
        len = min(remaining, pShtp->outMaxTransfer);

        // Stage one tranfer in the out buffer
        memcpy(pShtp->outTransfer+SHTP_HDR_LEN, pData+cursor, len);
        remaining -= len;
        cursor += len;

        // Add the header len
        len += SHTP_HDR_LEN;

        // Put the header in the out buffer
        pShtp->outTransfer[0] = len & 0xFF;
        pShtp->outTransfer[1] = (len >> 8) & 0xFF;
        if (continuation) {
            pShtp->outTransfer[1] |= 0x80;
        }
        pShtp->outTransfer[2] = chan;
        pShtp->outTransfer[3] = pShtp->chan[chan].nextOutSeq++;

        // Transmit
        int status = sh2_hal_tx(pShtp->unit, pShtp->outTransfer, len);
        if (status != SH2_OK) {
            // Error, throw away this cargo
            pShtp->txDiscards++;
            break;
        }

        // For the rest of this transmission, packets are continuations.
        continuation = true;
    }

    return status;
}




