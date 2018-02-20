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
 * Demo App for Hillcrest BNO080
 */


// Sensor Application
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sensor_app.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

// Number of sensor events that can be queued before dropping data.
// A good value would be twice the number of sensors enabled by the app.
#define SENSOR_EVENT_QUEUE_SIZE (6)

#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#endif

// Define this to produce DSF data for logging
// #define DSF_OUTPUT

// Define this to perform fimware update at startup.
// #define PERFORM_DFU

// Define this to use HMD-appropriate configuration.
// #define CONFIGURE_HMD

// Define this for calibration config appropriate for Robot Vaccuum Cleaners
// #define CONFIGURE_RVC

#ifdef PERFORM_DFU
#include "dfu_bno080.h"
#include "firmware.h"
#endif

#define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))
const float scaleDegToRad = 3.14159265358 / 180.0;

// --- Forward declarations -------------------------------------------

static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent);
static void onReset(void);

#ifdef DSF_OUTPUT
    static void printDsfHeaders(void);
    static void printDsf(const sh2_SensorEvent_t * event);
#endif

#ifndef DSF_OUTPUT
    static void reportProdIds(void);
    static void printEvent(const sh2_SensorEvent_t *pEvent);
#endif



// --- Private data ---------------------------------------------------

sh2_ProductIds_t prodIds;

SemaphoreHandle_t wakeSensorTask;

volatile bool resetPerformed = false;
volatile bool startedReports = false;

QueueHandle_t eventQueue;

// --- Public methods -------------------------------------------------


void demoTaskStart(const void * params)
{
    static uint32_t sensors = 0;
    sh2_SensorEvent_t sensorEvent;

    printf("\n\nHillcrest SH-2 Demo.\n");

    wakeSensorTask = xSemaphoreCreateBinary();
    eventQueue = xQueueCreate(SENSOR_EVENT_QUEUE_SIZE,
                              sizeof(sh2_SensorEvent_t));
    if (eventQueue == NULL) {
        printf("Error creating event queue.\n");
    }

#ifdef PERFORM_DFU
    // Perform DFU
    printf("Starting DFU process\n");
    int status = dfu(&firmware);
    
    printf("DFU completed with status: %d\n", status);

    if (status == SH2_OK) {
        // DFU Succeeded.  Need to pause a bit to let flash writes complete
        vTaskDelay(10);  // 10ms pause
    }
#endif

    resetPerformed = false;
    startedReports = false;

    // init SH2 layer
    sh2_initialize(eventHandler, NULL);
    
    // Register event listener
    sh2_setSensorCallback(sensorHandler, NULL);

    // wait for reset notification, or just go ahead after 100ms
    int waited = 0;
    while (!resetPerformed && (waited < 200)) {
        vTaskDelay(1);
        waited++;
    }

#ifdef DSF_OUTPUT
    // Print DSF file headers
    printDsfHeaders();
#else
    // Read and display BNO080 product ids
    reportProdIds();
#endif

    // Perform on-reset setup of BNO080
    onReset();

    // Process sensors forever
    while (1) {
        // Wait until something happens
        xSemaphoreTake(wakeSensorTask, portMAX_DELAY);
                             
        // Dequeue sensor events
        while (xQueueReceive(eventQueue, &sensorEvent, 0) == pdPASS) {
            sensors++;
#ifdef DSF_OUTPUT
            printDsf(&sensorEvent);
#else
            printEvent(&sensorEvent);
#endif
        }

        if (resetPerformed) {
            onReset();
        }
    }
}

// --- Private methods ----------------------------------------------

static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    if (pEvent->eventId == SH2_RESET) {
        printf("SH2 Reset.\n");

        // Signal main loop to handle this.
        resetPerformed = true;
        xSemaphoreGive(wakeSensorTask);
    }
}

static void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent)
{
    xQueueSend(eventQueue, pEvent, 0);
    xSemaphoreGive(wakeSensorTask);
}

#define GIRV_REF_6AG  (0x0207)  // 6 axis Game Rotation Vector 
#define GIRV_REF_9AGM (0x0204)  // 9 axis Absolute Rotation Vector
#define GIRV_SYNC_INTERVAL (10000)                     // sync interval: 10000 uS (100Hz)
#define GIRV_MAX_ERR FIX_Q(29, (30.0 * scaleDegToRad)) // max error: 30 degrees
#define GIRV_ALPHA FIX_Q(20, 0.303072543909142)        // pred param alpha
#define GIRV_BETA  FIX_Q(20, 0.113295896384921)        // pred param beta
#define GIRV_GAMMA FIX_Q(20, 0.002776219713054)        // pred param gamma

#ifdef CONFIGURE_HMD
    // Enable GIRV prediction for 28ms with 100Hz sync
    #define GIRV_PRED_AMT FIX_Q(10, 0.028)             // prediction amt: 28ms
#else
    // Disable GIRV prediction
    #define GIRV_PRED_AMT FIX_Q(10, 0.0)               // prediction amt: 0
#endif


static void configure(void)
{
    int status = SH2_OK;
    uint32_t config[7];

    // Note: The call to sh2_setFrs below updates a non-volatile FRS record
    // so it will remain in effect even after the sensor hub reboots.  It's not strictly
    // necessary to repeat this step every time the system starts up as we are doing
    // in this example code.
    
    // Configure prediction parameters for Gyro-Integrated Rotation Vector.
    // See section 4.3.24 of the SH-2 Reference Manual for a full explanation.
    // ...
    config[0] = GIRV_REF_6AG;           // Reference Data Type
    config[1] = (uint32_t)GIRV_SYNC_INTERVAL; // Synchronization Interval
    config[2] = (uint32_t)GIRV_MAX_ERR;  // Maximum error
    config[3] = (uint32_t)GIRV_PRED_AMT; // Prediction Amount
    config[4] = (uint32_t)GIRV_ALPHA;    // Alpha
    config[5] = (uint32_t)GIRV_BETA;     // Beta
    config[6] = (uint32_t)GIRV_GAMMA;    // Gamma
    status = sh2_setFrs(GYRO_INTEGRATED_RV_CONFIG, config, ARRAY_LEN(config));
    
    if (status != SH2_OK) {
        printf("Error: %d, from sh2_setFrs() in configure().\n", status);
    }

    // The sh2_setCalConfig does not update non-volatile storage.  This
    // only remains in effect until the sensor hub reboots.

#ifdef CONFIGURE_RVC
    // Enable planar calibration mode, which is designed for RVC applications
    status = sh2_setCalConfig(SH2_CAL_PLANAR);
#else
    // Enable dynamic calibration for A, G and M sensors
    status = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
#endif
    if (status != SH2_OK) {
        printf("Error: %d, from sh2_setCalConfig() in configure().\n", status);
    }
}

static void startReports(void)
{
    static sh2_SensorConfig_t config;
    int status;
    int sensorId;
        
    printf("Starting Sensor Reports.\n");

    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = 10000;  // microseconds (100Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000Hz)
    config.batchInterval_us = 0;

    sensorId = SH2_ROTATION_VECTOR;
    // sensorId = SH2_GYRO_INTEGRATED_RV;
    status = sh2_setSensorConfig(sensorId, &config);
    if (status != 0) {
        printf("Error while enabling sensor %d\n", sensorId);
    }
}

static void onReset(void)
{
    // Configure calibration config as we want it
    configure();

    // Start the flow of sensor reports
    startReports();
    
    // Toggle reset flag back to false
    resetPerformed = false;
}

#ifdef DSF_OUTPUT
static void printDsfHeaders(void)
{
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, ANG_POS_GLOBAL[rijk]{quaternion}, ANG_POS_ACCURACY[x]{rad}\n",
           SH2_ROTATION_VECTOR);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, GAME_ROTATION_VECTOR[rijk]{quaternion}\n",
           SH2_GAME_ROTATION_VECTOR);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_ACCELEROMETER[xyz]{adc units}\n",
           SH2_RAW_ACCELEROMETER);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_MAGNETOMETER[xyz]{adc units}\n",
           SH2_RAW_MAGNETOMETER);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_GYROSCOPE[xyz]{adc units}\n",
           SH2_RAW_GYROSCOPE);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, GYROSCOPE_CALIBRATED[xyz]{rad/s}\n",
           SH2_GYROSCOPE_CALIBRATED);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, GYROSCOPE_UNCALIBRATED[xyz]{rad/s}\n",
           SH2_GYROSCOPE_UNCALIBRATED);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, ACCELEROMETER[xyz]{m/s^2}\n",
           SH2_ACCELEROMETER);
    
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, MAG_FIELD[xyz]{uTesla}, STATUS[x]{enum}\n",
           SH2_MAGNETIC_FIELD_CALIBRATED);
    
    printf("+%d TIME[x]{s}, ANG_VEL_GYRO_RV[xyz]{rad/s}, ANG_POS_GYRO_RV[wxyz]{quaternion}\n",
           SH2_GYRO_INTEGRATED_RV);
}

static void printDsf(const sh2_SensorEvent_t * event)
{
    float t, r, i, j, k, acc_rad;
    float angVelX, angVelY, angVelZ;
    static uint32_t lastSequence[SH2_MAX_SENSOR_ID+1];  // last sequence number for each sensor
    sh2_SensorValue_t value;

    // Convert event to value
    sh2_decodeSensorEvent(&value, event);
    
    // Compute new sample_id
    uint8_t deltaSeq = value.sequence - (lastSequence[value.sensorId] & 0xFF);
    lastSequence[value.sensorId] += deltaSeq;

    // Get time as float
    t = value.timestamp / 1000000.0;
    
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf(".%d %0.6f, %d, %d, %d, %d\n",
                   SH2_RAW_ACCELEROMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z);
            break;
        
        case SH2_RAW_MAGNETOMETER:
            printf(".%d %0.6f, %d, %d, %d, %d\n",
                   SH2_RAW_MAGNETOMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawMagnetometer.x,
                   value.un.rawMagnetometer.y,
                   value.un.rawMagnetometer.z);
            break;
        
        case SH2_RAW_GYROSCOPE:
            printf(".%d %0.6f, %d, %d, %d, %d\n",
                   SH2_RAW_GYROSCOPE,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z);
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\n",
                   SH2_GYROSCOPE_CALIBRATED,
                   t,
                   lastSequence[value.sensorId],
                   value.un.gyroscope.x,
                   value.un.gyroscope.y,
                   value.un.gyroscope.z);
            break;

        case SH2_GYROSCOPE_UNCALIBRATED:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\n",
                   SH2_GYROSCOPE_UNCALIBRATED,
                   t,
                   lastSequence[value.sensorId],
                   value.un.gyroscopeUncal.x,
                   value.un.gyroscopeUncal.y,
                   value.un.gyroscopeUncal.z);
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %u\n",
                   SH2_MAGNETIC_FIELD_CALIBRATED,
                   t,
                   lastSequence[value.sensorId],
                   value.un.magneticField.x,
                   value.un.magneticField.y,
                   value.un.magneticField.z,
                   value.status & 0x3
                );
            break;
        
        case SH2_ACCELEROMETER:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\n",
                   SH2_ACCELEROMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.accelerometer.x,
                   value.un.accelerometer.y,
                   value.un.accelerometer.z);
            break;
        
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_rad = value.un.rotationVector.accuracy;
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
                   SH2_ROTATION_VECTOR,
                   t,
                   lastSequence[value.sensorId],
                   r, i, j, k,
                   acc_rad);
            break;

        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f\n",
                   SH2_ROTATION_VECTOR,
                   t,
                   lastSequence[value.sensorId],
                   r, i, j, k);
            break;
            
        case SH2_GYRO_INTEGRATED_RV:
            angVelX = value.un.gyroIntegratedRV.angVelX;
            angVelY = value.un.gyroIntegratedRV.angVelY;
            angVelZ = value.un.gyroIntegratedRV.angVelZ;
            r = value.un.gyroIntegratedRV.real;
            i = value.un.gyroIntegratedRV.i;
            j = value.un.gyroIntegratedRV.j;
            k = value.un.gyroIntegratedRV.k;
            printf(".%d %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
                   SH2_GYRO_INTEGRATED_RV,
                   t,
                   angVelX, angVelY, angVelZ,
                   r, i, j, k);
            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}
#endif

#ifndef DSF_OUTPUT
static void reportProdIds(void)
{
    int status;
    
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    
    if (status < 0) {
        printf("Error from sh2_getProdIds.\n");
        return;
    }

    // Report the results
    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
    }
}

static void printEvent(const sh2_SensorEvent_t * event)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;
    static int skip = 0;

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n", rc);
        return;
    }

    t = value.timestamp / 1000000.0;  // time in seconds.
    
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf("Raw acc: %d %d %d\n",
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y, value.un.rawAccelerometer.z);
            break;

        case SH2_ACCELEROMETER:
            printf("Acc: %f %f %f\n",
                   value.un.accelerometer.x,
                   value.un.accelerometer.y,
                   value.un.accelerometer.z);
            break;
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_deg = scaleRadToDeg * 
                value.un.rotationVector.accuracy;
            printf("%8.4f Rotation Vector: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\n",
                   t,
                   r, i, j, k, acc_deg);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf("%8.4f GRV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
                   t,
                   r, i, j, k);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            x = value.un.gyroscope.x;
            y = value.un.gyroscope.y;
            z = value.un.gyroscope.z;
            printf("%8.4f GYRO: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   t,
                   x, y, z);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            x = value.un.gyroscopeUncal.x;
            y = value.un.gyroscopeUncal.y;
            z = value.un.gyroscopeUncal.z;
            printf("%8.4f GYRO_UNCAL: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   t,
                   x, y, z);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            // These come at 1kHz, too fast to print all of them.
            // So only print every 10th one
            skip++;
            if (skip == 10) {
                skip = 0;
                r = value.un.gyroIntegratedRV.real;
                i = value.un.gyroIntegratedRV.i;
                j = value.un.gyroIntegratedRV.j;
                k = value.un.gyroIntegratedRV.k;
                x = value.un.gyroIntegratedRV.angVelX;
                y = value.un.gyroIntegratedRV.angVelY;
                z = value.un.gyroIntegratedRV.angVelZ;
                printf("%8.4f Gyro Integrated RV: "
                       "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\n",
                       t,
                       r, i, j, k,
                       x, y, z);
            }
            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}
#endif


