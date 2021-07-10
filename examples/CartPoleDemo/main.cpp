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

	b3PhysicsClientHandle sm;
	sm = b3ConnectPhysicsDirect();

	return 0;
}
