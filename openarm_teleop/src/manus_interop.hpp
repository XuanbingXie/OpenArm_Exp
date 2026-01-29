#ifndef MANUS_INTEROP_HPP
#define MANUS_INTEROP_HPP

#include "ManusSDK.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHECK_SDK_CALL(call)                   \
    do {                                       \
        SDKReturnCode ret = (call);            \
        if (ret != SDKReturnCode_Success) {    \
            fprintf(stderr, "SDK call failed (%s:%d): %d\n", __FILE__, __LINE__, ret); \
            exit(-1);                          \
        }                                      \
    } while (0)

std::atomic<bool> keep_manus_running(true);

float shared_thumb_index_distance = -1.0f;

float thumb_dist_to_gripper_joint_pos(float dist) {
    constexpr float offset = 3.651 / 100.0; // 3.651cm 
    constexpr float max_hand_dist = 11.873 / 100.0; // 11.873cm

    dist = std::max(0.f, dist-offset);
    return -std::min(1.f, dist/max_hand_dist);
}

void raw_device_data_callback(const RawDeviceDataInfo* const p_RawDeviceDataInfo) {
    if (!keep_manus_running) return;

    // Handle raw device data here
    for (uint32_t i = 0; i < p_RawDeviceDataInfo->rawDeviceDataCount; ++i) {
        RawDeviceData data;
        CHECK_SDK_CALL(CoreSdk_GetRawDeviceData(i, &data));

        if (data.sensorCount < 2) {
            continue; // Not enough sensor data
        }
        ManusVec3 thumb_position = data.sensorData[0].position;
        ManusVec3 index_position = data.sensorData[1].position;

        float thumb_index_distance = sqrtf(
            powf(thumb_position.x - index_position.x, 2) +
            powf(thumb_position.y - index_position.y, 2) +
            powf(thumb_position.z - index_position.z, 2)
        );

        shared_thumb_index_distance = thumb_index_distance;
    }
}

void initialize_manus_sdk() {
    CHECK_SDK_CALL(CoreSdk_InitializeIntegrated());
    CHECK_SDK_CALL(CoreSdk_SetSessionType(SessionType_CoreSDK));

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side_Right;
    vuh.up = AxisPolarity_PositiveZ;
    vuh.view = AxisView_XFromViewer;
    vuh.unitScale = 1.0f;
    CHECK_SDK_CALL(CoreSdk_InitializeCoordinateSystemWithVUH(vuh, 1));

    CoreSdk_RegisterCallbackForRawDeviceDataStream(raw_device_data_callback);

    ManusHost manus_host;
    CHECK_SDK_CALL(CoreSdk_GetAvailableHostsFound(&manus_host, 1));
    printf("Connecting to Manus Host: %s; IP Address: %s; Version: %d.%d.%d\n",
           manus_host.hostName,
           manus_host.ipAddress,
           manus_host.manusCoreVersion.major,
           manus_host.manusCoreVersion.minor,
           manus_host.manusCoreVersion.patch);
    CHECK_SDK_CALL(CoreSdk_ConnectToHost(manus_host));

    CHECK_SDK_CALL(CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto));
}

void shutdown_manus_sdk() {
    CoreSdk_Disconnect();
    CoreSdk_ShutDown();
}

#ifdef __cplusplus
}
#endif

#endif // MANUS_INTEROP_HPP
