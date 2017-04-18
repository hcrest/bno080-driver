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

/*
 * World Tare API Implementation.
 */

#include "worldTare.h"

#include <math.h>
#include <stdio.h>

#define PI (3.14159265358)
#define DEG2RAD(x) (x*PI/180.0)
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#define TOL (0.0005)

// ------------------------------------------------------------------------------
// Forward declarations
static float q2yaw(const Quaternion_t *q);
static int   yaw2q(float yaw, Quaternion_t *q);
static void  qMult(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *qResult);
static bool ut_q_yaw(void);

// ------------------------------------------------------------------------------
// Public API

// Creates a new tare state that, when applied, will result in rotation vector with same heading as qTo
// when applied to a rotation vector with heading of qFrom.
// qFrom and qTo should BOTH be rotation vectors that HAVE been converted with worldTare_apply.
int worldTare_setTareZ(const TareState_t *stateIn,
                       TareState_t *stateOut,
                       const Quaternion_t *qFrom,
                       const Quaternion_t *qTo)
{
    float yaw0;
    float yaw1;
    Quaternion_t ddq;

    // Return error if params are bad
    if ((stateIn == 0) || (stateOut == 0) || (qFrom == 0)) {
        return -1;
    }

    // Get headings from qTo_, qFrom
    if (qTo) {
        yaw0 = q2yaw(qTo);
    }
    else {
        yaw0 = 0.0;
    }
    
    yaw1 = q2yaw(qFrom);

    // Get quaternion that represents heading rotation yaw1 -> yaw2
    yaw2q(yaw0-yaw1, &ddq);

    // Apply delta to tare state
    qMult(&stateIn->q, &ddq, &stateOut->q);

    return 0;
}

int worldTare_clear(TareState_t *state)
{
    if (state == 0) return -1;
    
    state->q.w = 1.0;
    state->q.x = 0.0;
    state->q.y = 0.0;
    state->q.z = 0.0;

    return 0;
}

// Apply a world tare transformation on a rotation vector, qIn, resulting in
// an adjusted rotation vector, qOut.
int worldTare_apply(const TareState_t *state,
                    const Quaternion_t *qIn,
                    Quaternion_t *qOut)
{
    if ((state == 0) || (qIn == 0) || (qOut == 0)) return -1;
    
    qMult(&state->q, qIn, qOut);
    return 0;
}
    
bool worldTare_unitTest(void)
{
    bool status = true;

    status &= ut_q_yaw();

    return status;
}

// ------------------------------------------------------------------------------
// Utility functions

static bool inToleranceRad(float a, float b)
{
    float diff = b - a;
    if (diff > PI) diff -= 2.0*PI;
    if (diff < -PI) diff += 2.0*PI;
    if ((diff > TOL) || (diff < -TOL)) {
        return false;
    }
    
    return true;
}

static bool inTolerance(float a, float b)
{
    float diff = b - a;
    if ((diff > TOL) || (diff < -TOL)) {
        return false;
    }
    
    return true;
}

static bool ut_q_yaw(void)
{
    bool status = true;
    
    typedef struct {
        Quaternion_t qENU;
        float yaw;
    } TestPair_t;
    
    static const TestPair_t test[] = {
        // qENU[wxyz]                     , yaw
        {{0.0000,  0.0000,  0.0000,  1.0000}, DEG2RAD(-180)},
        {{0.2588,  0.0000,  0.0000,  0.9659}, DEG2RAD(-150)},
        {{0.5000,  0.0000,  0.0000,  0.8660}, DEG2RAD(-120)},
        {{0.7071,  0.0000,  0.0000,  0.7071}, DEG2RAD( -90)},
        {{0.8660,  0.0000,  0.0000,  0.5000}, DEG2RAD( -60)},
        {{0.9659,  0.0000,  0.0000,  0.2588}, DEG2RAD( -30)},
        {{1.0000,  0.0000,  0.0000, -0.0000}, DEG2RAD(   0)},
        {{0.9659,  0.0000,  0.0000, -0.2588}, DEG2RAD(  30)},
        {{0.8660,  0.0000,  0.0000, -0.5000}, DEG2RAD(  60)},
        {{0.7071,  0.0000,  0.0000, -0.7071}, DEG2RAD(  90)},
        {{0.5000,  0.0000,  0.0000, -0.8660}, DEG2RAD( 120)},
        {{0.2588,  0.0000,  0.0000, -0.9659}, DEG2RAD( 150)},
        {{0.0000,  0.0000,  0.0000, -1.0000}, DEG2RAD( 180)},
    };

    for (int n = 0; n < ARRAY_LEN(test); n++) {
        float yaw;
        Quaternion_t q;

        yaw = q2yaw(&test[n].qENU);
        yaw2q(test[n].yaw, &q);

        if (!inToleranceRad(yaw, test[n].yaw) ||
            !inTolerance(q.w, test[n].qENU.w) ||
            !inTolerance(q.x, test[n].qENU.x) ||
            !inTolerance(q.y, test[n].qENU.y) ||
            !inTolerance(q.z, test[n].qENU.z)) {
            status = false;
        }
    }

    return status;
}

static float q2yaw(const Quaternion_t *q)
{
    float num;
    float den;
    float yaw;
      
    // num = 2*y*x - 2*w*z
    num = 2.0 * q->y * q->x - 2.0 * q->w * q->z;
    
    // den = 2*w*w + 2*y*y - 1
    den = 2.0 * q->w * q->w + 2.0 * q->y * q->y - 1.0;
    
    yaw = atan2(num, den);
    
    return yaw;
}

static int yaw2q(float yaw, Quaternion_t *q)
{
    if (q == 0) return -1;

    q->w = cos(0.5 * yaw);
    q->x = 0.0;
    q->y = 0.0;
    q->z = -sin(0.5 * yaw);  

    return 0;
}

static void qMult(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *qResult)
{
    qResult->w =
        q1->w * q2->w
        - q1->x * q2->x
        - q1->y * q2->y
        - q1->z * q2->z;
    
    qResult->x =
        q1->w * q2->x
        + q1->x * q2->w
        + q1->y * q2->z
        - q1->z * q2->y;
    
    qResult->y =
        q1->w * q2->y
        - q1->x * q2->z
        + q1->y * q2->w
        + q1->z * q2->x;
    
    qResult->z =
        q1->w * q2->z
        + q1->x * q2->y
        - q1->y * q2->x
        + q1->z * q2->w;
}


