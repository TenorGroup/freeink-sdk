#pragma once

// Host-test stub for <freertos/task.h>. xTaskCreate runs the connection task on
// a real std::thread so connect() stays asynchronous exactly as on the device;
// vTaskDelete (called by BleKeyboardHost::end()) stops and joins it.

#include <freertos/FreeRTOS.h>
#include <stdint.h>

BaseType_t xTaskCreate(void (*fn)(void*), const char* name, uint32_t stackDepth, void* arg, UBaseType_t priority,
                       TaskHandle_t* outHandle);
void vTaskDelete(TaskHandle_t task);
void vTaskDelay(uint32_t ticks);
void xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, uint32_t ticksToWait);
