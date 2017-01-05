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

#ifndef DFU_H
#define DFU_H

#include <stdint.h>

#include "HcBin.h"

#ifdef __cplusplus
extern "C" {
#endif

// DFU Interface

/**
 * Run DFU Process.
 * @param unit Which SensorHub to operate with.
 * @param firmware Represents the firmware to be downloaded.
 * @return Err code from sh2_err.h indicating whether DFU process completed successfully
 */
int dfu(unsigned unit, const HcBin_t *firmware);

#ifdef __cplusplus
}   // end of extern "C"
#endif

#endif
