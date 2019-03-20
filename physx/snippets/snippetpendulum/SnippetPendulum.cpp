//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// ****************************************************************************
// This snippet demonstrates the use of articulations.
// ****************************************************************************

#include <iostream>
#include <fstream>

#include <ctype.h>
#include <vector>

#include "PxPhysicsAPI.h"

#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

extern void exitCallback(void);

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation		= NULL;
PxPhysics*				gPhysics		= NULL;

PxDefaultCpuDispatcher*	gDispatcher		= NULL;
PxScene*				gScene			= NULL;

PxMaterial*				gMaterial		= NULL;

PxPvd*                  gPvd			= NULL;

PxArticulation*			gArticulation	= NULL;

const float gravity	= 9.81f;
const float dt		= 1.0f/60.0f;
const int n_steps	= 10000;
const float distance= 1.0f;
const float sphereMass = 1.0f;
float inertia		= 2.0f;
int step_counter	= 0;
float posx[n_steps];
float posy[n_steps];
float time_vec[n_steps];

// Get position of last element
void get_position(int ii, float* posx, float* posy){
	PxArticulationLink* links[1];
	gArticulation->getLinks(links, 1, 0);
	PxVec3 pos = links[0]->getGlobalPose().p;
	posx[ii] = pos.x;
	posy[ii] = pos.y;
}

void log(){
	// Save data
	const static char* dir = "/home/xander/Google Drive/Thesis/src/pendulum_analysis/data/";
	char fname[40];
	sprintf(fname,"%spendulum_data",dir);
	std::cout << fname << std::endl;
	std::ofstream file;
	file.open(fname);
	const static int length_head = 6;
	file << "Length_head: "	<<  length_head << std::endl;
	file << "Mass: "		<<  sphereMass << std::endl;
	file << "Inertia:"		<<	inertia << std::endl;
	file << "Length: "		<<  distance << std::endl;
	file << "dt: "			<<  dt << std::endl;
	file << "gravity: "		<<  gravity << std::endl;
	// CSV header
	file << "time," << "x_coor," << "y_coor" << std::endl;
	for(int ii = 0; ii<step_counter-1; ii++){
		std::cout << ii << std::endl;
		file << time_vec[ii] << "," << posx[ii] << "," << posy[ii] << std::endl;
	}
	file.close();
}


void initPhysics(bool /*interactive*/)
{
	std::cout << "Press ESC to stop recording positions." << std::endl;
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -gravity, 0.0f);

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gArticulation = gPhysics->createArticulation();

	// Stabilization can create artefacts on jointed objects so we just disable it
	gArticulation->setStabilizationThreshold(0.0f);

	gArticulation->setMaxProjectionIterations(16);
	gArticulation->setSeparationTolerance(0.001f);

	const float radius = 0.1f;

	const PxVec3 initPos(0.0f, 0.0f, 0.0f);
	PxVec3 pos = initPos;
	pos.y = distance;
	PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
	PxArticulationLink* firstLink = NULL;
	PxArticulationLink* parent = NULL;

	// Create one link with an end mass
	std::cout << pos.x << std::endl;
	PxArticulationLink* link = gArticulation->createLink(parent, PxTransform(pos));
	firstLink = link;

	link->attachShape(*sphereShape);
	PxRigidBodyExt::setMassAndUpdateInertia(*link, sphereMass);
	PxVec3 inertia_tensor = link->getMassSpaceInertiaTensor();
	std::cout << "X: " << inertia_tensor.x << "Y: " << inertia_tensor.y << "Z: " << inertia_tensor.z << std::endl;
	inertia = inertia_tensor.x;
	//inertia_tensor.x = inertia;
	//inertia_tensor.x = inertia;
	//inertia_tensor.y = inertia;
	//inertia_tensor.z = inertia;
	//link->setMassSpaceInertiaTensor(inertia_tensor);
	//link->setMass(sphereMass);
	//std::cout << "New inertia tensor:" << std::endl << "X: " << inertia_tensor.x << "Y: " << inertia_tensor.y << "Z: " << inertia_tensor.z << std::endl;

	// Reduce sleep threshold
	PxReal sThresh = gArticulation->getSleepThreshold()/10;
	gArticulation->setSleepThreshold(sThresh);
	std::cout << sThresh << std::endl;

	gScene->addArticulation(*gArticulation);

	// Attach articulation to static world
	{
		PxShape* anchorShape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
		PxRigidStatic* anchor = PxCreateStatic(*gPhysics, PxTransform(initPos), *anchorShape);
		gScene->addActor(*anchor);
		PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), firstLink, PxTransform(PxVec3(0.0f,distance,0.0f)));
		PX_UNUSED(j);
	}

	PxReal compliance = 0.0f;
	PxU32 driveIterations = 1;
	PxArticulationDriveCache* dCache = gArticulation->createDriveCache(compliance, driveIterations);
	gArticulation->applyImpulse(link, *dCache, PxVec3(0.1f, 0.0f, 0.0f), PxVec3(0.0f, 0.0f, 0.0f));
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(dt);
	gScene->fetchResults(true);
	if(gArticulation->isSleeping()){
		std::cout << "Articulation is sleeping, stop simulation manually" << std::endl;
	}
	else if(step_counter<n_steps){
		get_position(step_counter, posx, posy);
		time_vec[step_counter] = step_counter*dt;
		step_counter++;
	} else {
		std::cout << "Max time reached, stop simulation manually" << std::endl;
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	// Save positions to file
	log();

	gArticulation->release();
	gScene->release();
	gDispatcher->release();
	gPhysics->release();
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	PxCloseExtensions();
	gFoundation->release();

	printf("SnippetArticulation done.\n");
}

void keyPress(unsigned char /*key*/, const PxTransform& /*camera*/)
{
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
