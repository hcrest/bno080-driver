/*
 * Copyright 2017 Hillcrest Laboratories, Inc.
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
 * @file worldTare.h
 * @author David Wheeler
 * @date 17 Feb 2017
 * @brief API Definition for world tare functions.
 *
 */

#ifndef WORLD_TARE_H
#define WORLD_TARE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        float w;
        float x;
        float y;
        float z;
    } Quaternion_t;

    typedef struct {
        Quaternion_t q;
    } TareState_t;

    // Creates a new tare state that, when applied, will result in rotation vector with same heading as qTo
    // when applied to a rotation vector with heading of qFrom.
    // qFrom and qTo should BOTH be rotation vectors that HAVE been converted with worldTare_apply.
    // @param stateIn Represents previous transformation in effect.
    // @param stateOut Output value: represents new transformation.
    // @param qFrom Rotation Vector with incorrect heading.
    // @param qTo   Rotation Vector with desired heading.
    // @retval      Status.  0 indicates success, negative value on error.
    int worldTare_setTareZ(const TareState_t *stateIn,
                           TareState_t *stateOut,
                           const Quaternion_t *qFrom,
                           const Quaternion_t *qTo);

    // Clear (or initialize) the world tare state.
    // @param stateIn Represents previous transformation in effect.
    // @param stateOut Output value: represents new transformation.
    // @param qFrom Rotation Vector with incorrect heading.
    // @param qTo   Rotation Vector with desired heading.
    // @retval      Status.  0 indicates success, negative value on error.
    int worldTare_clear(TareState_t *state);

    // Apply a world tare transformation on a rotation vector, qIn, resulting in
    // an adjusted rotation vector, qOut.
    // @param state Represents transformation to be applied.
    // @param qIn   Rotation Vector prior to tare operation.
    // @param qOut  Output value: Rotation Vector after tare operation.
    // @retval      Status.  0 indicates success, negative value on error.
    int worldTare_apply(const TareState_t *state,
                        const Quaternion_t *qIn,
                        Quaternion_t *qOut);
    
    // Perform unit tests on world tare module
    // @retval true if all tests passed.
    bool worldTare_unitTest(void);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif

