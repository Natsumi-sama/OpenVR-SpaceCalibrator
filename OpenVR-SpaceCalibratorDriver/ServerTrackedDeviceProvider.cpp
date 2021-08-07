#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include <string>
#include <sstream>

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof DeviceTransform);

	InjectHooks(this, pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform &newTransform)
{
	auto &tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	auto &tf = transforms[openVRID];
	if (tf.enabled)
	{
		pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];
	}
	return true;
}

void ServerTrackedDeviceProvider::SetPose(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (pose.poseIsValid) devicePoses[openVRID] = pose;
}

protocol::DevicePoses ServerTrackedDeviceProvider::GetDevicePoses()
{
	protocol::DevicePoses poses;
	uint32_t i = 0;
	for (std::pair<uint32_t, vr::DriverPose_t> p : devicePoses)
	{
		protocol::DevicePoses::DevicePose pose;
		pose.openVRID = p.first;
		vr::DriverPose_t devicePose = p.second;
		pose.qWorldFromDriverRotation = devicePose.qWorldFromDriverRotation;
		pose.vecWorldFromDriverTranslation[0] = devicePose.vecWorldFromDriverTranslation[0];
		pose.vecWorldFromDriverTranslation[1] = devicePose.vecWorldFromDriverTranslation[1];
		pose.vecWorldFromDriverTranslation[2] = devicePose.vecWorldFromDriverTranslation[2];
		pose.qDriverFromHeadRotation = devicePose.qDriverFromHeadRotation;
		pose.vecDriverFromHeadTranslation[0] = devicePose.vecDriverFromHeadTranslation[0];
		pose.vecDriverFromHeadTranslation[1] = devicePose.vecDriverFromHeadTranslation[1];
		pose.vecDriverFromHeadTranslation[2] = devicePose.vecDriverFromHeadTranslation[2];
		pose.qRotation = devicePose.qRotation;
		pose.vecPosition[0] = devicePose.vecPosition[0];
		pose.vecPosition[1] = devicePose.vecPosition[1];
		pose.vecPosition[2] = devicePose.vecPosition[2];
		poses.devicePoses[i] = pose;
		if (++i >= 8) break;
	}
	poses.length = i;
	return poses;
}
