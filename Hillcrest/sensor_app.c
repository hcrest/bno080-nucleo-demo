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
#include "shtp.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

// #define PERFORM_DFU
#ifdef PERFORM_DFU
#include "dfu.h"
#include "firmware.h"
#endif



// --- Forward declarations -------------------------------------------

static void reportProdIds(void);
static void startReports(void);
static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);
static void printEvent(const sh2_SensorEvent_t *pEvent);
static void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent);

// --- Private data ---------------------------------------------------

sh2_ProductIds_t prodIds;

SemaphoreHandle_t wakeSensorTask;

volatile bool resetPerformed = false;
volatile bool startedReports = false;

volatile bool sensorReceived = false;
sh2_SensorEvent_t sensorEvent;

// --- Public methods -------------------------------------------------


void demoTaskStart(const void * params)
{
    static uint32_t sensors = 0;

    printf("\n\nHillcrest SH-2 Demo.\n");

    wakeSensorTask = xSemaphoreCreateBinary();

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

    // init SHTP layer
    shtp_init();

    resetPerformed = false;
    startedReports = false;

    // init SH2 layer
    sh2_initialize(eventHandler, NULL);
    
    // Register event listener
    sh2_setSensorCallback(sensorHandler, NULL);

    while (!resetPerformed) {
        vTaskDelay(1);
    }

    // Read out product id
    reportProdIds();

    // Process sensors forever
    while (1) {
        // Wait until something happens
        xSemaphoreTake(wakeSensorTask, portMAX_DELAY);
                             
        if (sensorReceived) {
            sensorReceived = false;
            sensors++;
            printEvent(&sensorEvent);
        }
        if (resetPerformed) {
          resetPerformed = false;
          printf("Starting Sensor Reports.\n");

          // Enable reports from Rotation Vector.
          startReports();
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
    sensorEvent = *pEvent;
    sensorReceived = true;

    xSemaphoreGive(wakeSensorTask);
}

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
    for (int n = 0; n < SH2_NUM_PROD_ID_ENTRIES; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
    }

}

void startReports(void)
{
    static sh2_SensorConfig_t config;
    int status;
        
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = 10000;  // microseconds (100Hz)
    config.batchInterval_us = 0;

    status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (status != 0) {
        printf("Error while enabling RotationVector sensor: %d\n", status);
    }
    
    status = sh2_getSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (status != 0) {
        printf("Error while getting RotationVector config: %d\n", status);
    }
}

void printEvent(const sh2_SensorEvent_t * event)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;

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
                   "r:%5.3f i:%5.3f j:%5.3f k:%5.3f (acc: %5.3f deg)\n",
                   t,
                   r, i, j, k, acc_deg);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            r = value.un.gyroIntegratedRV.real;
            i = value.un.gyroIntegratedRV.i;
            j = value.un.gyroIntegratedRV.j;
            k = value.un.gyroIntegratedRV.k;
            x = value.un.gyroIntegratedRV.angVelX;
            y = value.un.gyroIntegratedRV.angVelX;
            z = value.un.gyroIntegratedRV.angVelX;
            printf("%8.4f Gyro Integrated RV: "
                   "r:%5.3f i:%5.3f j:%5.3f k:%5.3f x:%5.3f y:%5.3f z:%5.3f\n",
                   t,
                   r, i, j, k,
                   x, y, z);
            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}


