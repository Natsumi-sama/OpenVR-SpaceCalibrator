#include "stdafx.h"
#include "DllInterface.h"

static IPCClient Driver;

void Connect()
{
	Driver.Connect();
}

void SetDeviceTransform(uint32_t id, double x, double y, double z, double qx, double qy, double qz, double qw)
{
	vr::HmdVector3d_t pos;
	pos.v[0] = x;
	pos.v[1] = y;
	pos.v[2] = z;

	vr::HmdQuaternion_t rot;
	rot.x = qx;
	rot.y = qy;
	rot.z = qz;
	rot.w = qw;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, true, pos, rot };
	Driver.SendBlocking(req);
}

void ResetAndDisableDeviceTransform(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ };
	Driver.SendBlocking(req);
}

protocol::DevicePoses GetDevicePoses()
{
	protocol::Request req(protocol::RequestGetDevicePoses);
	protocol::Response res = Driver.SendBlocking(req);
	return res.devicePoses;
}

