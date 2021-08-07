#pragma once

#include "../Protocol.h"
#include "IPCClient.h"

extern "C"
{
	__declspec(dllexport) void Connect();
	__declspec(dllexport) void SetDeviceTransform(uint32_t id, double x, double y, double z, double qx, double qy, double qz, double qw);
	__declspec(dllexport) void ResetAndDisableDeviceTransform(uint32_t id);
	__declspec(dllexport) protocol::DevicePoses GetDevicePoses();
}
