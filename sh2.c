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
 * SH-2 API Implementation for Hillcrest BNO080
 */

#include <string.h>
#include <stdio.h>

#include "sh2.h"
#include "shtp.h"
#include "sh2_hal.h"
#include "sh2_err.h"

#include "sh2_util.h"

// Max length of an FRS record, words.
#define MAX_FRS_WORDS (72)

// Max length of sensorhub version string.
#define MAX_VER_LEN (16)

// Max number of report ids supported
#define SH2_MAX_REPORT_IDS (64)

// Tags for sensorhub app advertisements.
#define TAG_SH2_VERSION (0x80)
#define TAG_SH2_REPORT_LENGTHS (0x81)

// executable/device channel responses
#define EXECUTABLE_DEVICE_CMD_RESET (1)
#define EXECUTABLE_DEVICE_CMD_ON    (2)
#define EXECUTABLE_DEVICE_CMD_SLEEP (3)

// executable/device channel responses
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE (1)

// --- Private Data Types -------------------------------------------------

typedef int (sh2_OpStart_t)(unsigned unit);
typedef void (sh2_OpTxDone_t)(unsigned unit);
typedef void (sh2_OpRx_t)(unsigned unit, const uint8_t *payload, uint16_t len);

typedef struct sh2_Op_s {
	sh2_OpStart_t *start;
	sh2_OpTxDone_t *txDone;
	sh2_OpRx_t *rx;
} sh2_Op_t;

// Report definitions
// Bit fields for Feature Report flags
#define FEAT_CHANGE_SENSITIVITY_RELATIVE (1)
#define FEAT_CHANGE_SENSITIVITY_ABSOLUTE (0)
#define FEAT_CHANGE_SENSITIVITY_ENABLED  (2)
#define FEAT_CHANGE_SENSITIVITY_DISABLED (0)
#define FEAT_WAKE_ENABLED                (4)
#define FEAT_WAKE_DISABLED               (0)
#define FEAT_ALWAYS_ON_ENABLED           (8)
#define FEAT_ALWAYS_ON_DISABLED          (0)

// GET_FEATURE_REQ
#define SENSORHUB_GET_FEATURE_REQ    (0xFE)
typedef __packed struct {
	uint8_t reportId;
	uint8_t featureReportId;
} GetFeatureReq_t;

// SENSORHUB_SET_FEATURE_CMD
#define SENSORHUB_SET_FEATURE_CMD    (0xFD)
typedef __packed struct {
	uint8_t reportId;             // 0xFD
	uint8_t featureReportId;      // sensor id
	uint8_t flags;                // FEAT_... values
	uint16_t changeSensitivity;
	uint32_t reportInterval_uS;
	uint32_t batchInterval_uS;
	uint32_t sensorSpecific;
} SetFeatureReport_t;

// SENSORHUB_GET_FEATURE_RESP
#define SENSORHUB_GET_FEATURE_RESP   (0xFC)
typedef __packed struct {
	uint8_t reportId;
	uint8_t featureReportId;      // sensor id
	uint8_t flags;                // FEAT_... values
	uint16_t changeSensitivity;
	uint32_t reportInterval_uS;
	uint32_t batchInterval_uS;
	uint32_t sensorSpecific;
} GetFeatureResp_t;

#define SENSORHUB_BASE_TIMESTAMP_REF (0xFB)
typedef __packed struct {
	uint8_t reportId;
	uint32_t timebase;
} BaseTimestampRef_t;

#define SENSORHUB_TIMESTAMP_REBASE   (0xFA)
typedef __packed struct {
	uint8_t reportId;
	int32_t timebase;
} TimestampRebase_t;

// SENSORHUB_PROD_ID_REQ
#define SENSORHUB_PROD_ID_REQ        (0xF9)
typedef __packed struct {
	uint8_t reportId;  
	uint8_t reserved;
} ProdIdReq_t;

// SENSORHUB_PROD_ID_RESP
#define SENSORHUB_PROD_ID_RESP       (0xF8)
typedef __packed struct {
	uint8_t reportId;
	uint8_t resetCause;
	uint8_t swVerMajor;
	uint8_t swVerMinor;
	uint32_t swPartNumber;
	uint32_t swBuildNumber;
	uint16_t swVerPatch;
	uint8_t reserved0;
	uint8_t reserved1;
} ProdIdResp_t;

// SENSORHUB_FRS_WRITE_REQ
#define SENSORHUB_FRS_WRITE_REQ      (0xF7)
typedef __packed struct {
	uint8_t reportId;
	uint8_t reserved;
	uint16_t length;
	uint16_t frsType;
} FrsWriteReq_t;

// SENSORHUB_FRS_WRITE_DATA_REQ
#define SENSORHUB_FRS_WRITE_DATA_REQ (0xF6)
typedef __packed struct {
	uint8_t reportId;
	uint8_t reserved;
	uint16_t offset;
	uint32_t data0;
	uint32_t data1;
} FrsWriteDataReq_t;

// FRS write status values
#define FRS_WRITE_STATUS_RECEIVED (0)
#define FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE (1)
#define FRS_WRITE_STATUS_BUSY (2)
#define FRS_WRITE_STATUS_WRITE_COMPLETED (3)
#define FRS_WRITE_STATUS_READY (4)
#define FRS_WRITE_STATUS_FAILED (5)
#define FRS_WRITE_STATUS_NOT_READY (6) // data received when not in write mode
#define FRS_WRITE_STATUS_INVALID_LENGTH (7)
#define FRS_WRITE_STATUS_RECORD_VALID (8)
#define FRS_WRITE_STATUS_INVALID_RECORD (9)
#define FRS_WRITE_STATUS_DEVICE_ERROR (10)
#define FRS_WRITE_STATUS_READ_ONLY (11)

// SENSORHUB_FRS_WRITE_RESP
#define SENSORHUB_FRS_WRITE_RESP     (0xF5)
typedef __packed struct {
	uint8_t reportId;
	uint8_t status;
	uint16_t wordOffset;
} FrsWriteResp_t;

// RESP_FRS_READ_REQ
#define SENSORHUB_FRS_READ_REQ       (0xF4)
typedef __packed struct {
	uint8_t reportId;
	uint8_t reserved;
	uint16_t readOffset;
	uint16_t frsType;
	uint16_t blockSize;
} FrsReadReq_t;

// Get Datalen portion of len_status field
#define FRS_READ_DATALEN(x) ((x >> 4) & 0x0F)

// Get status portion of len_status field
#define FRS_READ_STATUS(x) ((x) & 0x0F)

// Status values
#define FRS_READ_STATUS_NO_ERROR                        0
#define FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE           1
#define FRS_READ_STATUS_BUSY                            2
#define FRS_READ_STATUS_READ_RECORD_COMPLETED           3
#define FRS_READ_STATUS_OFFSET_OUT_OF_RANGE             4
#define FRS_READ_STATUS_RECORD_EMPTY                    5
#define FRS_READ_STATUS_READ_BLOCK_COMPLETED            6
#define FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED 7
#define FRS_READ_STATUS_DEVICE_ERROR                    8

// SENSORHUB_FRS_READ_RESP
#define SENSORHUB_FRS_READ_RESP      (0xF3)
typedef __packed struct {
	uint8_t reportId;
	uint8_t len_status;  // See FRS_READ... macros above
	uint16_t wordOffset;
	uint32_t data0;
	uint32_t data1;
	uint16_t frsType;
	uint8_t reserved0;
	uint8_t reserved1;
} FrsReadResp_t;

// Command and Subcommand values
#define SH2_CMD_ERRORS                 1
#define SH2_CMD_COUNTS                 2
#define     SH2_COUNTS_GET_COUNTS          0
#define     SH2_COUNTS_CLEAR_COUNTS        1
#define SH2_CMD_TARE                   3
#define     SH2_TARE_TARE_NOW              0
#define     SH2_TARE_PERSIST_TARE          1
#define     SH2_TARE_SET_REORIENTATION     2
#define SH2_CMD_INITIALIZE             4
#define     SH2_INIT_SYSTEM                1
#define SH2_INIT_UNSOLICITED           0x80
#define SH2_CMD_FRS                    5
#define SH2_CMD_DCD                    6
#define SH2_CMD_ME_CAL                 7
#define SH2_CMD_SYNC                   8
#define     SH2_SYNC_SYNC_NOW              0
#define     SH2_SYNC_ENABLE_EXT_SYNC       1
#define     SH2_SYNC_DISABLE_EXT_SYNC      2
#define SH2_CMD_DCD_SAVE               9
#define SH2_CMD_GET_OSC_TYPE           10

// SENSORHUB_COMMAND_REQ
#define SENSORHUB_COMMAND_REQ        (0xF2)
typedef __packed struct {
	uint8_t reportId;
	uint8_t seq;
	uint8_t command;
	uint8_t p[9];
} CommandReq_t;

// SENSORHUB_COMMAND_RESP
#define SENSORHUB_COMMAND_RESP       (0xF1)
typedef __packed struct {
	uint8_t reportId;
	uint8_t seq;
	uint8_t command;
	uint8_t commandSeq;
	uint8_t respSeq;
	uint8_t r[11];
} CommandResp_t;

// SENSORHUB_FORCE_SENSOR_FLUSH
#define SENSORHUB_FORCE_SENSOR_FLUSH (0xF0)
typedef __packed struct {
	uint8_t reportId;
	uint8_t sensorId;
} ForceFlushReq_t;

// SENSORHUB_FLUSH_COMPLETED    
#define SENSORHUB_FLUSH_COMPLETED    (0xEF)
typedef __packed struct {
	uint8_t reportId;
	uint8_t sensorId;
} ForceFlushResp_t;

typedef struct sh2_s {
	unsigned unit;

	uint8_t controlChan;

	char version[MAX_VER_LEN+1];
	struct {
		uint8_t id;
		uint8_t len;
	} report[SH2_MAX_REPORT_IDS];

	uint32_t emptyPayloads;
	uint32_t unknownReportIds;
	uint32_t execBadPayload;

	sh2_ProductIds_t *pProdIds;

	bool advertDone;
	bool gotInitResp;
	bool calledResetCallback;
  
	sh2_EventCallback_t *eventCallback;
	void * eventCallbackCookie;

	sh2_SensorCallback_t *sensorCallback;
	void * sensorCallbackCookie;

	uint8_t nextCmdSeq;

	const sh2_Op_t *pOp;

	sh2_OpEvent_t opEvent;

	uint32_t frsData[MAX_FRS_WORDS];
	uint16_t frsDataLen;

	// Parameters and state information for the operation in progress
	union {
		struct {
			CommandReq_t req;
		} sendCmd;
		struct {
			sh2_SensorConfig_t *pConfig;
			sh2_SensorId_t sensorId;
		} getSensorConfig;
		struct {
			const sh2_SensorConfig_t *pConfig;
			sh2_SensorId_t sensorId;
		} setSensorConfig;
		struct {
			uint16_t frsType;
			uint32_t *pData;
			uint16_t *pWords;
			uint16_t lastOffset;
			sh2_SensorMetadata_t *pMetadata;
		} getFrs;
		struct {
			uint16_t frsType;
			uint32_t *pData;
			uint16_t words;
			uint16_t offset;
		} setFrs;
		struct {
			uint8_t seq;
			uint8_t severity;
			sh2_ErrorRecord_t *pErrors;
			uint16_t *pNumErrors;
			uint16_t errsRead;
		} getErrors;
		struct {
			uint8_t seq;
			sh2_SensorId_t sensorId;
			sh2_Counts_t *pCounts;
		} getCounts;
		struct {
			uint8_t seq;
		} reinit;
		struct {
			uint8_t seq;
		} saveDcdNow;
		struct {
			uint8_t sensors;
			uint8_t seq;
		} calConfig;
		struct {
			sh2_SensorId_t sensorId;
		} forceFlush;
        struct {
			uint8_t seq;
            sh2_OscType_t *pOscType;
        } getOscType;
	} opData;

} sh2_t;

// --- Forward Declarations -----------------------------------------------
static int16_t toQ14(double x);
static void setupCmdParams(unsigned unit, uint8_t cmd, uint8_t p[9]);
static void setupCmd1(unsigned unit, uint8_t cmd, uint8_t p0);
	
static void executableAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val);
static void executableDeviceHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);

static void sensorhubAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val);
static void sensorhubControlHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
static void sensorhubInputNormalHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
static void sensorhubInputWakeHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
static void sensorhubInputGyroRvHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);

static uint8_t getReportLen(sh2_t *pSh2, uint8_t reportId);
	
// SH-2 transaction phases
static int opStart(unsigned unit,
                   const sh2_Op_t *pOp);
static void opTxDone(unsigned unit);
static void opRx(unsigned unit, const uint8_t *payload, uint16_t len);
static int opCompleted(unsigned unit, int status);

static uint64_t touSTimestamp(uint32_t hostInt, int32_t referenceDelta, uint16_t delay);

// --- Private Data -------------------------------------------------------
static sh2_t sh2[SH2_UNITS];

// SH-2 Transaction handlers

// Operation to Send a Command, No response
static int sendCmdStart(unsigned unit);
static void sendCmdTxDone(unsigned unit);
const sh2_Op_t sendCmdOp = {
	.start = sendCmdStart,
	.txDone = sendCmdTxDone,
};

// Operation to get product id
static int getProdIdStart(unsigned unit);
static void getProdIdRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t getProdIdOp = {
	.start = getProdIdStart,
	.rx = getProdIdRx,
};

// getSensorConfig Operation
static int getSensorConfigStart(unsigned unit);
static void getSensorConfigRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t getSensorConfigOp = {
	.start = getSensorConfigStart,
	.rx = getSensorConfigRx,
};

// setSensorConfig Operation
static int setSensorConfigStart(unsigned unit);
static void setSensorConfigTxDone(unsigned unit);
const sh2_Op_t setSensorConfigOp = {
	.start = setSensorConfigStart,
	.txDone = setSensorConfigTxDone,
};

// get FRS Operation
static int getFrsStart(unsigned unit);
static void getFrsRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t getFrsOp = {
	.start = getFrsStart,
	.rx = getFrsRx,
};

// set FRS Operation
static int setFrsStart(unsigned unit);
static void setFrsRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t setFrsOp = {
	.start = setFrsStart,
	.rx = setFrsRx,
};

// get errors operation
static int getErrorsStart(unsigned unit);
static void getErrorsRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t getErrorsOp = {
	.start = getErrorsStart,
	.rx = getErrorsRx,
};

// get counts operation
static int getCountsStart(unsigned unit);
static void getCountsRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t getCountsOp = {
	.start = getCountsStart,
	.rx = getCountsRx,
};

// reinitialize operation
static int reinitStart(unsigned unit);
static void reinitRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t reinitOp = {
	.start = reinitStart,
	.rx = reinitRx,
};

// save dcd now operation
static int saveDcdNowStart(unsigned unit);
static void saveDcdNowRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t saveDcdNowOp = {
	.start = saveDcdNowStart,
	.rx = saveDcdNowRx,
};

// cal config operation
static int calConfigStart(unsigned unit);
static void calConfigRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t calConfigOp = {
	.start = calConfigStart,
	.rx = calConfigRx,
};

// force flush operation
static int forceFlushStart(unsigned unit);
static void forceFlushRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t forceFlushOp = {
	.start = forceFlushStart,
	.rx = forceFlushRx,
};

// getOscType Operation
static int getOscTypeStart(unsigned unit);
static void getOscTypeRx(unsigned unit, const uint8_t *payload, uint16_t len);
const sh2_Op_t getOscTypeOp = {
	.start = getOscTypeStart,
	.rx = getOscTypeRx,
};

// --- Public API ---------------------------------------------------------

// sh2_init
int sh2_initialize(unsigned unit, sh2_EventCallback_t *eventCallback, void *resetCookie)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	pSh2->controlChan = 0xFF;  // An invalid value since we don't know yet.

	pSh2->emptyPayloads = 0;
	pSh2->unknownReportIds = 0;

	pSh2->advertDone = false;
	pSh2->gotInitResp = false;
	pSh2->calledResetCallback = false;
  
	pSh2->eventCallback = eventCallback;
	pSh2->eventCallbackCookie = resetCookie;
	pSh2->sensorCallback = 0;
	pSh2->sensorCallbackCookie = 0;

	pSh2->pOp = 0;
	pSh2->opEvent.unit = unit;
	
	for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
		pSh2->report[n].id = 0;
		pSh2->report[n].len = 0;
	}
  
	pSh2->nextCmdSeq = 0;

	// Register SH2 handlers
	shtp_listenAdvert(unit, "sensorhub", sensorhubAdvertHdlr, (void *)unit);
	shtp_listenChan(unit, "sensorhub", "control", sensorhubControlHdlr, (void *)unit);
	shtp_listenChan(unit, "sensorhub", "inputNormal", sensorhubInputNormalHdlr, (void *)unit);
	shtp_listenChan(unit, "sensorhub", "inputWake", sensorhubInputWakeHdlr, (void *)unit);
	shtp_listenChan(unit, "sensorhub", "inputGyroRv", sensorhubInputGyroRvHdlr, (void *)unit);

	pSh2->execBadPayload = 0;

	// Register EXECUTABLE handlers
	shtp_listenAdvert(unit, "executable", executableAdvertHdlr, (void *)unit);
	shtp_listenChan(unit, "executable", "device", executableDeviceHdlr, (void *)unit);
    

	return SH2_OK;
}

int sh2_setSensorCallback(unsigned unit, sh2_SensorCallback_t *callback, void *cookie)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	pSh2->sensorCallback = callback;
	pSh2->sensorCallbackCookie = cookie;

	return SH2_OK;
}

int sh2_getProdIds(unsigned unit,
                   sh2_ProductIds_t *pProdIds)
{
	sh2_t *pSh2 = &sh2[unit];
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	if (pProdIds) {
		pProdIds->nextEntry = 0;
	}
	pSh2->pProdIds = pProdIds;
	return opStart(unit,
                   &getProdIdOp);
}

int sh2_getSensorConfig(unsigned unit,
                        sh2_SensorId_t sensorId, sh2_SensorConfig_t *config)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	pSh2->opData.getSensorConfig.sensorId = sensorId;
	pSh2->opData.getSensorConfig.pConfig = config;
	return opStart(unit,
                   &getSensorConfigOp);
}

int sh2_setSensorConfig(unsigned unit,
                        sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

    // Set up operation
	pSh2->opData.setSensorConfig.sensorId = sensorId;
	pSh2->opData.setSensorConfig.pConfig = pConfig;

	return opStart(unit,
                   &setSensorConfigOp);
}

const static struct {
	sh2_SensorId_t sensorId;
	uint16_t recordId;
} sensorToRecordMap[] = {
	{ SH2_RAW_ACCELEROMETER,            FRS_ID_META_RAW_ACCELEROMETER },
	{ SH2_ACCELEROMETER,                FRS_ID_META_ACCELEROMETER },
	{ SH2_LINEAR_ACCELERATION,          FRS_ID_META_LINEAR_ACCELERATION },
	{ SH2_GRAVITY,                      FRS_ID_META_GRAVITY },
	{ SH2_RAW_GYROSCOPE,                FRS_ID_META_RAW_GYROSCOPE },
	{ SH2_GYROSCOPE_CALIBRATED,         FRS_ID_META_GYROSCOPE_CALIBRATED },
	{ SH2_GYROSCOPE_UNCALIBRATED,       FRS_ID_META_GYROSCOPE_UNCALIBRATED },
	{ SH2_RAW_MAGNETOMETER,             FRS_ID_META_RAW_MAGNETOMETER },
	{ SH2_MAGNETIC_FIELD_CALIBRATED,    FRS_ID_META_MAGNETIC_FIELD_CALIBRATED },
	{ SH2_MAGNETIC_FIELD_UNCALIBRATED,  FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED },
	{ SH2_ROTATION_VECTOR,              FRS_ID_META_ROTATION_VECTOR },
	{ SH2_GAME_ROTATION_VECTOR,         FRS_ID_META_GAME_ROTATION_VECTOR },
	{ SH2_GEOMAGNETIC_ROTATION_VECTOR,  FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR },
	{ SH2_PRESSURE,                     FRS_ID_META_PRESSURE },
	{ SH2_AMBIENT_LIGHT,                FRS_ID_META_AMBIENT_LIGHT },
	{ SH2_HUMIDITY,                     FRS_ID_META_HUMIDITY },
	{ SH2_PROXIMITY,                    FRS_ID_META_PROXIMITY },
	{ SH2_TEMPERATURE,                  FRS_ID_META_TEMPERATURE },
	{ SH2_TAP_DETECTOR,                 FRS_ID_META_TAP_DETECTOR },
	{ SH2_STEP_DETECTOR,                FRS_ID_META_STEP_DETECTOR },
	{ SH2_STEP_COUNTER,                 FRS_ID_META_STEP_COUNTER },
	{ SH2_SIGNIFICANT_MOTION,           FRS_ID_META_SIGNIFICANT_MOTION },
	{ SH2_STABILITY_CLASSIFIER,         FRS_ID_META_STABILITY_CLASSIFIER },
	{ SH2_SHAKE_DETECTOR,               FRS_ID_META_SHAKE_DETECTOR },
	{ SH2_FLIP_DETECTOR,                FRS_ID_META_FLIP_DETECTOR },
	{ SH2_PICKUP_DETECTOR,              FRS_ID_META_PICKUP_DETECTOR },
	{ SH2_STABILITY_DETECTOR,           FRS_ID_META_STABILITY_DETECTOR },
	{ SH2_PERSONAL_ACTIVITY_CLASSIFIER, FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER },
	{ SH2_SLEEP_DETECTOR,               FRS_ID_META_SLEEP_DETECTOR },
	{ SH2_TILT_DETECTOR,                FRS_ID_META_TILT_DETECTOR },
	{ SH2_POCKET_DETECTOR,              FRS_ID_META_POCKET_DETECTOR },
	{ SH2_CIRCLE_DETECTOR,              FRS_ID_META_CIRCLE_DETECTOR },
};

int sh2_getMetadata(unsigned unit,
                    sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// pData must be non-null
	if (pData == 0) return SH2_ERR_BAD_PARAM;
  
	// Convert sensorId to metadata recordId
	int i;
	for (i = 0; i < ARRAY_LEN(sensorToRecordMap); i++) {
		if (sensorToRecordMap[i].sensorId == sensorId) {
			break;
		}
	}
	if (i >= ARRAY_LEN(sensorToRecordMap)) {
		// no match was found
		return SH2_ERR_BAD_PARAM;
	}
	uint16_t recordId = sensorToRecordMap[i].recordId;
	
	// Set up an FRS read operation
	pSh2->opData.getFrs.frsType = recordId;
	pSh2->opData.getFrs.pData = pSh2->frsData;
	pSh2->frsDataLen = ARRAY_LEN(pSh2->frsData);
	pSh2->opData.getFrs.pWords = &pSh2->frsDataLen;
	pSh2->opData.getFrs.lastOffset = 0;
	pSh2->opData.getFrs.pMetadata = pData;

	return opStart(unit,
                   &getFrsOp);
}

int sh2_getFrs(unsigned unit,
               uint16_t recordId, uint32_t *pData, uint16_t *words)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// Store params for this op
	pSh2->opData.getFrs.frsType = recordId;
	pSh2->opData.getFrs.pData = pData;
	pSh2->opData.getFrs.pWords = words;
	pSh2->opData.getFrs.lastOffset = 0;
	pSh2->opData.getFrs.pMetadata = 0;

	return opStart(unit,
                   &getFrsOp);
}

int sh2_setFrs(unsigned unit,
               uint16_t recordId, uint32_t *pData, uint16_t words)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	pSh2->opData.setFrs.frsType = recordId;
	pSh2->opData.setFrs.pData = pData;
	pSh2->opData.setFrs.words = words;

	return opStart(unit,
                   &setFrsOp);
}

int sh2_getErrors(unsigned unit,
                  uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *numErrors)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	pSh2->opData.getErrors.severity = severity;
	pSh2->opData.getErrors.pErrors = pErrors;
	pSh2->opData.getErrors.pNumErrors = numErrors;
	
	return opStart(unit,
                   &getErrorsOp);
}

int sh2_getCounts(unsigned unit,
                  sh2_SensorId_t sensorId, sh2_Counts_t *pCounts)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	pSh2->opData.getCounts.sensorId = sensorId;
	pSh2->opData.getCounts.pCounts = pCounts;
	
	return opStart(unit,
                   &getCountsOp);
}

int sh2_clearCounts(unsigned unit,
                    sh2_SensorId_t sensorId)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	setupCmd1(unit, SH2_CMD_COUNTS, SH2_COUNTS_CLEAR_COUNTS);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_setTareNow(unsigned unit,
                   uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t basis)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	setupCmd1(unit, SH2_CMD_TARE, SH2_TARE_TARE_NOW);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_clearTare(unsigned unit)
{
	uint8_t p[9];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	memset(p, 0, sizeof(p));
	p[0] = SH2_TARE_SET_REORIENTATION;

	setupCmdParams(unit, SH2_CMD_TARE, p);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_persistTare(unsigned unit)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	setupCmd1(unit, SH2_CMD_TARE, SH2_TARE_PERSIST_TARE);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_setReorientation(unsigned unit,
                         sh2_Quaternion_t *orientation)
{
	uint8_t p[9];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	p[0] = SH2_TARE_SET_REORIENTATION;
	writeu16(&p[1], toQ14(orientation->x));
	writeu16(&p[3], toQ14(orientation->y));
	writeu16(&p[5], toQ14(orientation->z));
	writeu16(&p[7], toQ14(orientation->w));

	setupCmdParams(unit, SH2_CMD_TARE, p);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_reinitialize(unsigned unit)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	return opStart(unit,
                   &reinitOp);
}

int sh2_saveDcdNow(unsigned unit)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	return opStart(unit,
                   &saveDcdNowOp);
}

int sh2_getOscType(unsigned unit,
                   sh2_OscType_t *pOscType)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

    pSh2->opData.getOscType.pOscType = pOscType;
	return opStart(unit,
                   &getOscTypeOp);
}


int sh2_setCalConfig(unsigned unit,
                  uint8_t sensors)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	pSh2->opData.calConfig.sensors = sensors;

	return opStart(unit,
                   &calConfigOp);
}

int sh2_syncRvNow(unsigned unit)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	setupCmd1(unit, SH2_CMD_SYNC, SH2_SYNC_SYNC_NOW);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_setExtSync(unsigned unit,
                  bool enabled)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	setupCmd1(unit, SH2_CMD_SYNC,
	          enabled ? SH2_SYNC_ENABLE_EXT_SYNC : SH2_SYNC_DISABLE_EXT_SYNC);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_setDcdAutoSave(unsigned unit,
                    bool enabled)
{
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	setupCmd1(unit, SH2_CMD_DCD_SAVE, enabled ? 0 : 1);
	return opStart(unit,
                   &sendCmdOp);
}

int sh2_flush(unsigned unit,
              sh2_SensorId_t sensorId)
{
	sh2_t *pSh2 = &sh2[unit];

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	// Set up flush operation
	pSh2->opData.forceFlush.sensorId = sensorId;

	return opStart(unit,
                   &forceFlushOp);
}

// --- Private utility functions --------------------------------------------------------------

static int16_t toQ14(double x)
{
	int16_t retval = (int16_t)(x * (2<<14));
	
	return retval;
}

// Send a command with parameters from p
static void setupCmdParams(unsigned unit, uint8_t cmd, uint8_t p[9])
{
	sh2_t *pSh2 = &sh2[unit];

	// Set up request
	pSh2->opData.sendCmd.req.reportId = SENSORHUB_COMMAND_REQ;
	pSh2->opData.sendCmd.req.seq = pSh2->nextCmdSeq++;
	pSh2->opData.sendCmd.req.command = cmd;
	memcpy(&pSh2->opData.sendCmd.req.p, p,
	       sizeof(pSh2->opData.sendCmd.req.p));
}

// Set up command to send
static void setupCmd1(unsigned unit, uint8_t cmd, uint8_t p0)
{
	uint8_t p[9];

	memset(p, 0, sizeof(p));
	p[0] = p0;
	setupCmdParams(unit, cmd, p);
}

static void sensorhubAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val)
{
	unsigned unit = (unsigned)cookie;
	sh2_t *pSh2 = &sh2[unit];

	switch (tag) {
	case TAG_SH2_VERSION:
		strcpy(pSh2->version, (const char *)val);
		break;

	case TAG_SH2_REPORT_LENGTHS:
	{
		uint8_t reports = len/2;
		if (reports > SH2_MAX_REPORT_IDS) {
			// Hub gave us more report lengths than we can store!
			reports = SH2_MAX_REPORT_IDS;
		}
		
		for (int n = 0; n < reports; n++) {
			pSh2->report[n].id = val[n*2];
			pSh2->report[n].len = val[n*2 + 1];
		}
		break;
	}
	
	case 0:
	{
		// 0 tag indicates end of advertisements for this app
		// At this time, the SHTP layer can give us our channel number.
		pSh2->controlChan = shtp_chanNo(unit, "sensorhub", "control");

		pSh2->advertDone = true;
		break;
	}
		
	default:
		break;
	}
}

static void sensorhubControlHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
	unsigned unit = (unsigned)cookie;
	sh2_t *pSh2 = &sh2[unit];
	uint16_t cursor = 0;
	uint32_t count = 0;
	CommandResp_t * pResp = 0;
    sh2_AsyncEvent_t event;
	
	if (len == 0) {
		pSh2->emptyPayloads++;
		return;
	}

	while (cursor < len) {
		// Get next report id
		count++;
		uint8_t reportId = payload[cursor];

		// Determine report length
		uint8_t reportLen = 0;
		for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
			if (pSh2->report[n].id == reportId) {
				reportLen = pSh2->report[n].len;
				break;
			}
		}
		if (reportLen == 0) {
			// An unrecognized report id
			pSh2->unknownReportIds++;
			return;
		}
		else {
			// Check for unsolicited initialize response or FRS change response
			if (reportId == SENSORHUB_COMMAND_RESP) {
				pResp = (CommandResp_t *)(payload+cursor);
				if ((pResp->command == (SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED)) &&
				    (pResp->r[1] == SH2_INIT_SYSTEM)) {
					// This is an unsolicited INIT message.
					// Is it time to call reset callback?
					pSh2->gotInitResp = true;
				}
				if (pResp->command == (SH2_CMD_FRS | SH2_INIT_UNSOLICITED))
                {
					// This is an unsolicited FRS change message
                    event.eventId = SH2_FRS_CHANGE;
                    event.frsType = pResp->r[1] + (pResp->r[2] << 8);
                    if (pSh2->eventCallback) {
                        pSh2->eventCallback(pSh2->eventCallbackCookie, &event);
                    }
				}
			}

			// Hand off to operation in progress, if any
			opRx(unit, payload+cursor, reportLen);
			cursor += reportLen;
		}
	}
}

static void sensorhubInputHdlr(sh2_t *pSh2, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
	sh2_SensorEvent_t event;
	uint16_t cursor = 0;

    uint32_t referenceDelta;

	referenceDelta = 0;

	while (cursor < len) {
		// Get next report id
		uint8_t reportId = payload[cursor];

		// Determine report length
		uint8_t reportLen = getReportLen(pSh2, reportId);
		if (reportLen == 0) {
			// An unrecognized report id
			pSh2->unknownReportIds++;
			return;
		}
		else {
			if (reportId == SENSORHUB_BASE_TIMESTAMP_REF) {
				const BaseTimestampRef_t *rpt = (const BaseTimestampRef_t *)(payload+cursor);
				
				// store base timestamp reference
				referenceDelta = -rpt->timebase;
			}
			else if (reportId == SENSORHUB_TIMESTAMP_REBASE) {
				const TimestampRebase_t *rpt = (const TimestampRebase_t *)(payload+cursor);

				referenceDelta += rpt->timebase;
			}
			else {
                uint8_t *pReport = payload+cursor;
                uint16_t delay = ((pReport[2] & 0xFC) << 6) + pReport[3];
                event.timestamp_uS = touSTimestamp(timestamp, referenceDelta, delay);
                event.reportId = reportId;
                event.pReport = pReport;
				event.len = reportLen;
				if (pSh2->sensorCallback != 0) {
					pSh2->sensorCallback(pSh2->sensorCallbackCookie, &event);
				}
			}
			cursor += reportLen;
		}
	}
}

static void sensorhubInputNormalHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
	unsigned unit = (unsigned)cookie;
	sh2_t *pSh2 = &sh2[unit];
	
	sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputWakeHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
	unsigned unit = (unsigned)cookie;
	sh2_t *pSh2 = &sh2[unit];
	
	sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputGyroRvHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
	unsigned unit = (unsigned)cookie;
	sh2_t *pSh2 = &sh2[unit];

	sh2_SensorEvent_t event;
	uint16_t cursor = 0;

    uint8_t reportId = SH2_GYRO_INTEGRATED_RV;
    uint8_t reportLen = getReportLen(pSh2, reportId);

    while (cursor < len) {
        event.timestamp_uS = timestamp;
        event.reportId = reportId;
        event.pReport = payload+cursor;
        event.len = reportLen;

        if (pSh2->sensorCallback != 0) {
            pSh2->sensorCallback(pSh2->sensorCallbackCookie, &event);
        }

        cursor += reportLen;
    }
}

static uint8_t getReportLen(sh2_t *pSh2, uint8_t reportId)
{
    for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
        if (pSh2->report[n].id == reportId) {
            return pSh2->report[n].len;
        }
    }

    return 0;
}

// SH-2 transaction phases
static int opStart(unsigned unit,
                   const sh2_Op_t *pOp)
{
	sh2_t *pSh2 = &sh2[unit];

	// return error if another operation already in progress
	if (pSh2->pOp) return SH2_ERR_OP_IN_PROGRESS;

	// Establish this operation as the new operation in progress
	pSh2->pOp = pOp;
	int rc = pOp->start(unit);  // Call start method
	if (rc != SH2_OK) {
		// Operation failed to start
		
		// Unregister this operation
		pSh2->pOp = 0;
	}

    // Block the calling thread until the operation completes.
    sh2_hal_block(unit);

	// Clear operation in progress
	pSh2->pOp = 0;
	
	return rc;
}

static void opTxDone(unsigned unit)
{
	sh2_t *pSh2 = &sh2[unit];

	if ((pSh2->pOp != 0) && (pSh2->pOp->txDone != 0)) {
		pSh2->pOp->txDone(unit);  // Call txDone method
	}
}

static void opRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];

	if ((pSh2->pOp != 0) && (pSh2->pOp->rx != 0)) {
		pSh2->pOp->rx(unit, payload, len);  // Call receive method
	}
}

static int opCompleted(unsigned unit, int status)
{
	sh2_t *pSh2 = &sh2[unit];

	// Record status
	pSh2->opEvent.status = status;

    // Block the calling thread until the operation completes.
    sh2_hal_unblock(unit);

	return SH2_OK;
}

// Produce 64-bit microsecond timestamp for a sensor event
static uint64_t touSTimestamp(uint32_t hostInt, int32_t referenceDelta, uint16_t delay)
{
	static uint32_t lastHostInt = 0;
	static uint32_t rollovers = 0;
	uint64_t timestamp;

	// Count times hostInt timestamps rolled over to produce upper bits
	if (hostInt < lastHostInt) {
		rollovers++;
	}
	lastHostInt = hostInt;
	
	timestamp = ((uint64_t)rollovers << 32);
	timestamp += hostInt + (referenceDelta + delay) * 100;

	return timestamp;
}

// --- Operation: Send Command (no response expected.) ----------------------

static int sendCmdStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	
	// Send request
	rc = shtp_send(unit, pSh2->controlChan,
                   (uint8_t *)&pSh2->opData.sendCmd.req,
                   sizeof(pSh2->opData.sendCmd.req));
    opTxDone(unit);

	return rc;
}

static void sendCmdTxDone(unsigned unit)
{
	opCompleted(unit, SH2_OK);
}
	
// --- Operation: Get Product Id ----------------------

// Get Product ID Op handler
static int getProdIdStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	ProdIdReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	// Set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_PROD_ID_REQ;
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void getProdIdRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	ProdIdResp_t *resp = (ProdIdResp_t *)payload;
	
	// skip this if it isn't the product id response.
	if (resp->reportId != SENSORHUB_PROD_ID_RESP) return;

	// Store this product id, if we can
	sh2_ProductIds_t *pProdIds = pSh2->pProdIds;
	
	if (pProdIds) {
		// Store the product id response
		if (pProdIds->nextEntry < SH2_NUM_PROD_ID_ENTRIES) {
			sh2_ProductId_t *pProdId = &pProdIds->entry[pProdIds->nextEntry];
			
			pProdId->resetCause = resp->resetCause;
			pProdId->swVersionMajor = resp->swVerMajor;
			pProdId->swVersionMinor = resp->swVerMinor;
			pProdId->swPartNumber = resp->swPartNumber;
			pProdId->swBuildNumber = resp->swBuildNumber;
			pProdId->swVersionPatch = resp->swVerPatch;
			pProdId->reserved0 = resp->reserved0;
			pProdId->reserved1 = resp->reserved1;

			pProdIds->nextEntry++;
		}
	}

	// Complete this operation if there is no storage for more product ids
	if ((pProdIds == 0) ||
	    (pProdIds->nextEntry == SH2_NUM_PROD_ID_ENTRIES)) {
		opCompleted(unit, SH2_OK);
	}

	return;
}

// --- Operation: Get Sensor Config ----------------------

static int getSensorConfigStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	GetFeatureReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_GET_FEATURE_REQ;
	req.featureReportId = pSh2->opData.getSensorConfig.sensorId;
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void getSensorConfigRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	GetFeatureResp_t *resp = (GetFeatureResp_t *)payload;
	sh2_SensorConfig_t *pConfig;
	
	// skip this if it isn't the response we're waiting for.
	if (resp->reportId != SENSORHUB_GET_FEATURE_RESP) return;
	if (resp->featureReportId != pSh2->opData.getSensorConfig.sensorId) return;

	// Copy out data
	pConfig = pSh2->opData.getSensorConfig.pConfig;
	
	pConfig->changeSensitivityEnabled = ((resp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED) != 0);
	pConfig->changeSensitivityRelative = ((resp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE) != 0);
	pConfig->wakeupEnabled = ((resp->flags & FEAT_WAKE_ENABLED) != 0);
	pConfig->alwaysOnEnabled = ((resp->flags & FEAT_ALWAYS_ON_ENABLED) != 0);
	pConfig->changeSensitivity = resp->changeSensitivity;
	pConfig->reportInterval_us = resp->reportInterval_uS;
	pConfig->batchInterval_us = resp->batchInterval_uS;
	pConfig->sensorSpecific = resp->sensorSpecific;

	// Complete this operation
	opCompleted(unit, SH2_OK);

	return;
}

static int setSensorConfigStart(unsigned unit)
{
	sh2_t *pSh2 = &sh2[unit];
	SetFeatureReport_t req;
	uint8_t flags = 0;
	int rc;
	sh2_SensorConfig_t *pConfig = pSh2->opData.getSensorConfig.pConfig;
	
	if (pConfig->changeSensitivityEnabled)  flags |= FEAT_CHANGE_SENSITIVITY_ENABLED;
	if (pConfig->changeSensitivityRelative) flags |= FEAT_CHANGE_SENSITIVITY_RELATIVE;
	if (pConfig->wakeupEnabled)             flags |= FEAT_WAKE_ENABLED;
	if (pConfig->alwaysOnEnabled)           flags |= FEAT_ALWAYS_ON_ENABLED;

	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_SET_FEATURE_CMD;
	req.featureReportId = pSh2->opData.setSensorConfig.sensorId;
	req.flags = flags;
	req.changeSensitivity = pConfig->changeSensitivity;
	req.reportInterval_uS = pConfig->reportInterval_us;
	req.batchInterval_uS = pConfig->batchInterval_us;
	req.sensorSpecific = pConfig->sensorSpecific;

	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void setSensorConfigTxDone(unsigned unit)
{
	// complete immediately
	opCompleted(unit, SH2_OK);
}

// --- get frs operation ------------------------------------

static int getFrsStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	FrsReadReq_t req;

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_FRS_READ_REQ;
	req.reserved = 0;
	req.readOffset = 0; // read from start
	req.frsType = pSh2->opData.getFrs.frsType;
	req.blockSize = *(pSh2->opData.getFrs.pWords);

	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void stuffMetadata(sh2_SensorMetadata_t *pData, uint32_t *frsData)
{
	// Populate the sensorMetadata structure with results
	pData->meVersion        = (frsData[0] >> 0) & 0xFF;
	pData->mhVersion        = (frsData[0] >> 8) & 0xFF;
	pData->shVersion        = (frsData[0] >> 16) & 0xFF;
	pData->range            = frsData[1];
	pData->resolution       = frsData[2];
	pData->power_mA         = (frsData[3] >> 0) & 0xFFFF;    // 16.10 forma = Xt
	pData->revision         = (frsData[3] >> 16) & 0xFFFF;
	pData->minPeriod_uS     = frsData[4];
	pData->fifoMax          = (frsData[5] >> 0) & 0xFFFF;
	pData->fifoReserved     = (frsData[5] >> 16) & 0xFFFF;
	pData->batchBufferBytes = (frsData[6] >> 0) & 0xFFFF;;
	pData->vendorIdLen      = (frsData[6] >> 16) & 0xFFFF;
	strcpy(pData->vendorId, ""); // init with empty string in case vendorIdLen == 0
	if (pData->revision == 0) {
		memcpy(pData->vendorId, (uint8_t *)&frsData[7], pData->vendorIdLen);
	}
	else if (pData->revision == 1) {
		pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
		pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
		memcpy(pData->vendorId, (uint8_t *)&frsData[8], pData->vendorIdLen);
	}
	else if (pData->revision == 2) {
		pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
		pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
		pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
		memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
		int vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
		memcpy(pData->vendorId, (uint8_t *)&frsData[vendorIdOffset],
		       pData->vendorIdLen);
	}
	else {
		// Unrecognized revision!
	}
}

static void getFrsRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	FrsReadResp_t *resp = (FrsReadResp_t *)payload;
	uint8_t status;

	// skip this if it isn't the response we're looking for
	if (resp->reportId != SENSORHUB_FRS_READ_RESP) return;

	// Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
	status = FRS_READ_STATUS(resp->len_status);
	if ((status == FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE) ||
	    (status == FRS_READ_STATUS_BUSY) ||
	    (status == FRS_READ_STATUS_OFFSET_OUT_OF_RANGE) ||
	    (status == FRS_READ_STATUS_DEVICE_ERROR)
		) {
		// Operation failed
		opCompleted(unit, SH2_ERR_HUB);
		return;
	}

	if (status == FRS_READ_STATUS_RECORD_EMPTY) {
		// Empty record, return zero length.
		*(pSh2->opData.getFrs.pWords) = 0;
		opCompleted(unit, SH2_OK);
	}

	// Store the contents from this response
	uint16_t offset = resp->wordOffset;
	
	// store first word, if we have room
	if (offset <= *(pSh2->opData.getFrs.pWords)) {
		pSh2->opData.getFrs.pData[offset] = resp->data0;
		pSh2->opData.getFrs.lastOffset = offset;
	}

	// store second word if there is one and we have room
	if ((FRS_READ_DATALEN(resp->len_status) == 2)  &&
	    (offset <= *(pSh2->opData.getFrs.pWords))) {
		pSh2->opData.getFrs.pData[offset+1] = resp->data1;
		pSh2->opData.getFrs.lastOffset = offset+1;
	}

	// If read is done, complete the operation
	if ((status == FRS_READ_STATUS_READ_RECORD_COMPLETED) ||
	    (status == FRS_READ_STATUS_READ_BLOCK_COMPLETED) ||
	    (status == FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED)) {
		*(pSh2->opData.getFrs.pWords) = pSh2->opData.getFrs.lastOffset+1;

        // If this was performed from getMetadata, copy the results into pMetadata.
		if (pSh2->opData.getFrs.pMetadata != 0) {
			stuffMetadata(pSh2->opData.getFrs.pMetadata, pSh2->opData.getFrs.pData);
		}

		opCompleted(unit, SH2_OK);
	}

	return;
}

// --- set frs operation ------------------------------------

static int setFrsStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	FrsWriteReq_t req;

	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	pSh2->opData.setFrs.offset = 0;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_FRS_WRITE_REQ;
	req.reserved = 0;
	req.length = pSh2->opData.setFrs.words;
	req.frsType = pSh2->opData.getFrs.frsType;

	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void setFrsRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	FrsWriteResp_t *resp = (FrsWriteResp_t *)payload;
	FrsWriteDataReq_t req;
	uint8_t status;
	bool sendMoreData = false;
	bool completed = false;
	int rc = SH2_OK;

	// skip this if it isn't the response we're looking for.
	if (resp->reportId != SENSORHUB_FRS_WRITE_RESP) return;

	// Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
	status = resp->status;
	switch(status) {
	case FRS_WRITE_STATUS_RECEIVED:
	case FRS_WRITE_STATUS_READY:
		sendMoreData = true;
		break;
	case FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE:
	case FRS_WRITE_STATUS_BUSY:
	case FRS_WRITE_STATUS_FAILED:
	case FRS_WRITE_STATUS_NOT_READY:
	case FRS_WRITE_STATUS_INVALID_LENGTH:
	case FRS_WRITE_STATUS_INVALID_RECORD:
	case FRS_WRITE_STATUS_DEVICE_ERROR:
	case FRS_WRITE_STATUS_READ_ONLY:
		rc = SH2_ERR_HUB;
		completed = true;
		break;
	case FRS_WRITE_STATUS_WRITE_COMPLETED:
		// Successful completion
		rc = SH2_OK;
		completed = true;
		break;
	case FRS_WRITE_STATUS_RECORD_VALID:
		// That's nice, keep waiting
		break;
	}

	// if we should send more data, do it.
	if (sendMoreData &&
	    (pSh2->opData.setFrs.offset < pSh2->opData.setFrs.words)) {
		uint16_t offset = pSh2->opData.setFrs.offset;
		
		memset(&req, 0, sizeof(req));
		req.reportId = SENSORHUB_FRS_WRITE_DATA_REQ;
		req.reserved = 0;
		req.offset = offset;
		req.data0 = pSh2->opData.setFrs.pData[offset++];
		if (offset < pSh2->opData.setFrs.words) {
		    req.data1 = pSh2->opData.setFrs.pData[offset++];
	    } else {
		    req.data1 = 0;
	    }
		pSh2->opData.setFrs.offset = offset;
		
		rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
        opTxDone(unit);

	}

	// if the operation is done or has to be aborted, complete it
	if (completed) {
		opCompleted(unit, rc);
	}

	return;
}

// --- Operation: get errors --------------

static int getErrorsStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// Create a command sequence number for this command
	pSh2->opData.getErrors.seq = pSh2->nextCmdSeq++;
	pSh2->opData.getErrors.errsRead = 0;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_COMMAND_REQ;
	req.seq = pSh2->opData.getErrors.seq;
	req.command = SH2_CMD_ERRORS;
	req.p[0] = pSh2->opData.getErrors.severity;
	
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);
	
	return rc;
}

static void getErrorsRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	CommandResp_t *resp = (CommandResp_t *)payload;
	
	// skip this if it isn't the right response
	if (resp->reportId != SENSORHUB_COMMAND_RESP) return;
	if (resp->command != SH2_CMD_ERRORS) return;
	if (resp->commandSeq != pSh2->opData.getErrors.seq) return;


	if (resp->r[2] == 255) {
		// No error to report, operation is complete
		*(pSh2->opData.getErrors.pNumErrors) = pSh2->opData.getErrors.errsRead;
		opCompleted(unit, SH2_OK);
	} else {
		// Copy data for invoker.
		unsigned index = pSh2->opData.getErrors.errsRead;
		if (index < *(pSh2->opData.getErrors.pNumErrors)) {
			// We have room for this one.
			pSh2->opData.getErrors.pErrors[index].severity = resp->r[0];
			pSh2->opData.getErrors.pErrors[index].sequence = resp->r[1];
			pSh2->opData.getErrors.pErrors[index].source = resp->r[2];
			pSh2->opData.getErrors.pErrors[index].error = resp->r[3];
			pSh2->opData.getErrors.pErrors[index].module = resp->r[4];
			pSh2->opData.getErrors.pErrors[index].code = resp->r[5];

			pSh2->opData.getErrors.errsRead++;
		}
	}

	return;
}

// --- Operation: get counts --------------

static int getCountsStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// Create a command sequence number for this command
	pSh2->opData.getCounts.seq = pSh2->nextCmdSeq++;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_COMMAND_REQ;
	req.seq = pSh2->opData.getCounts.seq;
	req.command = SH2_CMD_COUNTS;
	req.p[0] = SH2_COUNTS_GET_COUNTS;
	req.p[1] = pSh2->opData.getCounts.sensorId;
	
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void getCountsRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	CommandResp_t *resp = (CommandResp_t *)payload;
	
	// skip this if it isn't the right response
	if (resp->reportId != SENSORHUB_COMMAND_RESP) return;
	if (resp->command != SH2_CMD_COUNTS) return;
	if (resp->commandSeq != pSh2->opData.getCounts.seq) return;

	// Store results
	if (resp->respSeq == 0) {
		pSh2->opData.getCounts.pCounts->offered = readu32(&resp->r[3]);
		pSh2->opData.getCounts.pCounts->accepted = readu32(&resp->r[7]);
	}
	else {
		pSh2->opData.getCounts.pCounts->on = readu32(&resp->r[3]);
		pSh2->opData.getCounts.pCounts->attempted = readu32(&resp->r[7]);
	}
	
	// Complete this operation if we've received last response
	if (resp->respSeq == 1) {
		opCompleted(unit, SH2_OK);
	}

	return;
}

// --- Operation: reinitialize --------------

static int reinitStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// Create a command sequence number for this command
	pSh2->opData.reinit.seq = pSh2->nextCmdSeq++;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_COMMAND_REQ;
	req.seq = pSh2->opData.reinit.seq;
	req.command = SH2_CMD_INITIALIZE;
	req.p[0] = SH2_INIT_SYSTEM;
	
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void reinitRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	CommandResp_t *resp = (CommandResp_t *)payload;
	
	// skip this if it isn't the right response
	if (resp->reportId != SENSORHUB_COMMAND_RESP) return;
	if (resp->command != SH2_CMD_INITIALIZE) return;
	if (resp->commandSeq != pSh2->opData.reinit.seq) return;

	// If return status is error, return error to invoker
	int rc = SH2_OK;
	if (resp->r[0] != 0) {
		rc = SH2_ERR_HUB;
	}
	
	// Complete this operation
	opCompleted(unit, rc);

	return;
}

// --- Operation: save dcd now --------------

static int saveDcdNowStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// Create a command sequence number for this command
	pSh2->opData.saveDcdNow.seq = pSh2->nextCmdSeq++;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_COMMAND_REQ;
	req.seq = pSh2->opData.saveDcdNow.seq;
	req.command = SH2_CMD_DCD;
	
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void saveDcdNowRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	CommandResp_t *resp = (CommandResp_t *)payload;
	
	// skip this if it isn't the right response
	if (resp->reportId != SENSORHUB_COMMAND_RESP) return;
	if (resp->command != SH2_CMD_DCD) return;
	if (resp->commandSeq != pSh2->opData.saveDcdNow.seq) return;

	// If return status is error, return error to invoker
	int rc = SH2_OK;
	if (resp->r[0] != 0) {
		rc = SH2_ERR_HUB;
	}
	
	// Complete this operation
	opCompleted(unit, rc);

	return;
}

// --- Operation: cal config --------------

static int calConfigStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;

	// Create a command sequence number for this command
	pSh2->opData.calConfig.seq = pSh2->nextCmdSeq++;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_COMMAND_REQ;
	req.seq = pSh2->opData.calConfig.seq;
	req.command = SH2_CMD_ME_CAL;
	req.p[0] = (pSh2->opData.calConfig.sensors & SH2_CAL_ACCEL) ? 1 : 0; // accel cal
	req.p[1] = (pSh2->opData.calConfig.sensors & SH2_CAL_GYRO)  ? 1 : 0; // gyro cal
	req.p[2] = (pSh2->opData.calConfig.sensors & SH2_CAL_MAG)   ? 1 : 0; // mag cal
	
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void calConfigRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandResp_t *resp = (CommandResp_t *)payload;
	
	// skip this if it isn't the right response
	if (resp->reportId != SENSORHUB_COMMAND_RESP) return;
	if (resp->command != SH2_CMD_ME_CAL) return;
	if (resp->commandSeq != pSh2->opData.calConfig.seq) return;

	// If return status is error, return error to invoker
	if (resp->r[0] != 0) {
		rc = SH2_ERR_HUB;
	}
	
	// Complete this operation
	opCompleted(unit, rc);

	return;
}

// --- Operation: force flush --------------

static int forceFlushStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	ForceFlushReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_FORCE_SENSOR_FLUSH;
	req.sensorId = pSh2->opData.forceFlush.sensorId;
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void forceFlushRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	ForceFlushResp_t *resp = (ForceFlushResp_t *)payload;
	
	// skip this if it isn't the flush completed response for this sensor
	if (resp->reportId != SENSORHUB_FLUSH_COMPLETED) return;
	if (resp->sensorId != pSh2->opData.forceFlush.sensorId) return;

	// Complete this operation
	opCompleted(unit, SH2_OK);

	return;
}

// --- Operation: get oscillator type --------------

static int getOscTypeStart(unsigned unit)
{
	int rc = SH2_OK;
	sh2_t *pSh2 = &sh2[unit];
	CommandReq_t req;
	
	// Validate unit
	if (unit >= ARRAY_LEN(sh2)) return SH2_ERR_BAD_PARAM;
  
	// Create a command sequence number for this command
	pSh2->opData.getOscType.seq = pSh2->nextCmdSeq++;
	
	// set up request to issue
	memset(&req, 0, sizeof(req));
	req.reportId = SENSORHUB_COMMAND_REQ;
	req.seq = pSh2->opData.getOscType.seq;
	req.command = SH2_CMD_GET_OSC_TYPE;
	
	rc = shtp_send(unit, pSh2->controlChan, (uint8_t *)&req, sizeof(req));
    opTxDone(unit);

	return rc;
}

static void getOscTypeRx(unsigned unit, const uint8_t *payload, uint16_t len)
{
	sh2_t *pSh2 = &sh2[unit];
	CommandResp_t *resp = (CommandResp_t *)payload;
	sh2_OscType_t *pOscType;
	
	// skip this if it isn't the response we're waiting for.
	if (resp->reportId != SENSORHUB_COMMAND_RESP) return;
	if (resp->command != SH2_CMD_GET_OSC_TYPE) return;
	if (resp->commandSeq != pSh2->opData.getOscType.seq) return;

	// Read out data
	pOscType = pSh2->opData.getOscType.pOscType;
	*pOscType = (sh2_OscType_t)resp->r[0];

	// Complete this operation
	opCompleted(unit, SH2_OK);

	return;
}

static void executableAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val)
{
	switch (tag) {
	default:
		// Ignore unknown tags
		// (And there are no known tags for this app.)
		break;
	}
}

static void executableDeviceHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
	unsigned unit = (unsigned)cookie;
	sh2_t *pSh2 = &sh2[unit];
    sh2_AsyncEvent_t event;

	// Discard if length is bad
	if (len != 1) {
		pSh2->execBadPayload++;
		return;
	}
	
	switch (payload[0]) {
	case EXECUTABLE_DEVICE_RESP_RESET_COMPLETE:
        // Notify client that reset is complete.
        event.eventId = SH2_RESET;
        if (pSh2->eventCallback) {
            pSh2->eventCallback(pSh2->eventCallbackCookie, &event);
        }
		break;
	default:
		pSh2->execBadPayload++;
		break;
	}
}
