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

/**
 * @file sh2_hal.h
 * @author David Wheeler
 * @date 18 Nov 2016
 * @brief Hardware Adaptation Layer API for SensorHub-2 (and BNO080)
 */

#ifndef SH2_HAL_H
#define SH2_HAL_H

#include <stdint.h>
#include <stdbool.h>

// sh2_hal_impl.h should be provided by system designer
#include "sh2_hal_impl.h"

#ifndef SH2_HAL_MAX_TRANSFER
#error SH2_HAL_MAX_TRANSFER must be defined by sh2_hal_impl.h
#endif

#ifdef __cplusplus
extern "C" {
#endif

    // Callback signature for data receive events
    // Important: t_us is the 32-bit timestamp, in microseconds, associated with
    //            the INTN signal assertion.
    typedef void sh2_rxCallback_t(void * cookie, uint8_t *pData, uint32_t len, uint32_t t_us);

    // Reset an SH-2 module (into DFU mode, if flag is true)
    // The onRx callback function is registered with the HAL at the same time.
    // sh2_hal_reset() MUST be called at least once before sh2_tx or sh2_rx are used.
    int sh2_hal_reset(bool dfuMode,
                      sh2_rxCallback_t *onRx,
                      void *cookie);

    // Send data to SH-2.
    // Call may return without blocking before transfer is complete.
    int sh2_hal_tx(uint8_t *pData, uint32_t len);

    // Read len bytes from device into pData.
    // Blocks until transfer is complete.
    // This function is necessary when INTN does not generate
    // read operations.  e.g. for DFU support.
    //
    // (In normal operation, the HAL will respond to INTN by
    // automatically reading the device.  The data produced will be
    // delivered to the client via the onRx callback)
    int sh2_hal_rx(uint8_t *pData, uint32_t len);

    // Block the calling thread until unblock occurs.
    // (If tx, rx are implemented in a blocking fashion, these should be no-operations.)
    int sh2_hal_block(void);
    int sh2_hal_unblock(void);

#ifdef __cplusplus
}    // end of extern "C"
#endif

// #ifdef SH2_HAL_H
#endif
