//
// Created by ohta on 2021/07/10.
//

#include <iostream>

#include "../SharedMemory/SharedMemoryPublic.h"
#include "../SharedMemory/PhysicsClientC_API.h"

void getJointState(b3PhysicsClientHandle& sm, int bodyUniqueId)
{
	struct b3JointSensorState sensorState;

	// Display current state before applying action
	b3SharedMemoryCommandHandle cmd_handle = b3RequestActualStateCommandInit(sm, bodyUniqueId);
	b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
	int status_type = b3GetStatusType(status_handle);
	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		std::cout << "getJointState failed.\n"
				  << std::endl;
		return;
	}
	cmd_handle = b3RequestActualStateCommandInit(sm, bodyUniqueId);
	status_handle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
	status_type = b3GetStatusType(status_handle);
	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		std::cout << "getJointState failed." << std::endl;
		return;
	}
	int jointIndex = 0;
	if (b3GetJointState(sm, status_handle, jointIndex, &sensorState))
	{
		std::cout << "Joint position " << sensorState.m_jointPosition;
		std::cout << " velocity " << sensorState.m_jointVelocity << std::endl;
	}
	else
	{
		std::cout << "getJointState failed (2)." << std::endl;
	}
}

int main(int argc, char* argv[])
{
	int method = eCONNECT_DIRECT;
	printf("method = %d\n", method);

	// Directly connect to physics engine
	b3PhysicsClientHandle sm = b3ConnectPhysicsDirect();

	// Load CartPole URDF model
	int flags = 0;
	const char* urdfFileName = "../../build_cmake/examples/pybullet/pybullet_data/cartpole.urdf";
	b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);
	b3LoadUrdfCommandSetFlags(command, flags);
	printf("Loading %s...\n", urdfFileName);
	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	int statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_URDF_LOADING_COMPLETED)
	{
		printf("Status type is %d, not %d. Cannot load URDF file.\n",
			   statusType, int(CMD_URDF_LOADING_COMPLETED));
		return -1;
	}
	int bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	std::cout << "The bodyUniqueID is " << bodyUniqueId << std::endl;

	// Show current state before applying action
	getJointState(sm, bodyUniqueId);

	// Apply dummy torque
	int controlMode = CONTROL_MODE_VELOCITY;
	int jointIndex = 0;  // 0 for cart and 1 for pole
	struct b3JointInfo info;
	double targetVelocity = 0.0;
	double kd = 1.0;
	double force = 100000.0;
	b3SharedMemoryCommandHandle commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);
	b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

	b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex, targetVelocity);
	b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
	b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	printf("Current status is %d\n", int(statusHandle->unused));

	// Step simulation
	if (b3CanSubmitCommand(sm))
	{
		statusHandle = b3SubmitClientCommandAndWaitStatus(
			sm, b3InitStepSimulationCommand(sm));
		statusType = b3GetStatusType(statusHandle);
	}

	return 0;
}
