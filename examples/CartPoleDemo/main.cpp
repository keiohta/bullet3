//
// Created by ohta on 2021/07/10.
//

#include <stdio.h>

#include "../SharedMemory/SharedMemoryPublic.h"
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"

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
	printf("The bodyUniqueID is %d\n", bodyUniqueId);

	return 0;
}
