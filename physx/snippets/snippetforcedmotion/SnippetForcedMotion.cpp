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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// ****************************************************************************
// This snippet demonstrates the use of articulations.
// ****************************************************************************

#include <iostream>

#include <ctype.h>
#include <vector>

#include "PxPhysicsAPI.h"

#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

#include <ctime>
#include <fstream>
#include <math.h>

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation		= NULL;
PxPhysics*				gPhysics		= NULL;

PxDefaultCpuDispatcher*	gDispatcher		= NULL;
PxScene*				gScene			= NULL;

PxMaterial*				gMaterial		= NULL;

PxPvd*                  gPvd			= NULL;

PxArticulationReducedCoordinate*		gArticulation	= NULL;
PxArticulationJointReducedCoordinate*	gDriveJoint		= NULL;
PxFixedJoint* baseJoint									= NULL;
PxArticulationLink*						lastLink		= NULL;
PxArticulationLink*						anchor			= NULL;

const float gravity										= 9.81f;
const int								nLinks			= 60; // Near max amount of links
const int								additionalLinks	= 2;
const float								totalLength		= 32.0f;		// [m]
const float 							distance		= totalLength/nLinks;
const float 							mass			= 0.5f;		// [kg/m]

const PxReal							dt				= 1.0f / 60.f;	// [s]
const double							time_wait		= 5;			// [s]
const double							time_record		= 8;			// [s]
const float freq										= 8.0f;			// [Hz]
const float amplitude									= 0.8 ;		// [m]

const char* path = "/home/xander/Google Drive/Thesis/src/forced_motion_analysis/data/data";
std::fstream file;

// Get positions of first and last tether element
void getPositions(PxVec3* basePos, PxVec3* endPos){
	PxArticulationLink* links[nLinks+additionalLinks];
	const int startIndex = 0;
	gArticulation->getLinks(links, nLinks+additionalLinks, startIndex);
	// First link is fixed, second link is driving
	*basePos = links[1]->getGlobalPose().p;
	*endPos  = links[nLinks+additionalLinks-1]->getGlobalPose().p;
}

void initFile(){
	// Save data
	std::cout << path << std::endl;
	std::ofstream file;
	file.open(path, std::ios_base::out);
	const static int length_head = 6;
	file << "Length_head: "	<<  length_head << std::endl;
	file << "Timestep: "	<<  dt << std::endl;
	file << "Number: "		<<  nLinks << std::endl;
	file << "Distance: "	<<  distance << std::endl;
	file << "Frequency: "	<<  freq << std::endl;
	file << "Amplitude: "	<<  amplitude << std::endl;

	// CSV header
	file << "time," << "base_x_coor," << "base_y_coor," << "base_z_coor," << "end_x_coor," << "end_y_coor," << "end_z_coor" << std::endl;
	file.close();
}

// Save basePos and Endpos by appending to file
void logPositions(float t, PxVec3* basePos, PxVec3* endPos){
	file.open(path, std::ios_base::app);
	// Save data
	file << t << "," << basePos->x << "," << basePos->y << "," << basePos->z << "," << endPos->x << "," << endPos->y << "," << endPos->z << std::endl;
	file.close();
}

void applyDrag(){
	const PxU32 startIndex	= additionalLinks;
	PxArticulationLink* links[nLinks];
	gArticulation->getLinks(links, nLinks, startIndex);
	static float CD = 0.001f;
	for(PxU32 ii = 0; ii<nLinks; ii++){
		PxVec3 vel = links[ii]->getLinearVelocity();
		PxVec3 drag = vel*CD;
		links[ii]->addForce(-drag);
	}
}

void createAttachment()
{
	// Create base...
	PxArticulationLink* base = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f)));

	PxShape* shape = gPhysics->createShape(PxSphereGeometry(0.1f), *gMaterial);
	// Attach base articulation to static world
	{
		PxRigidStatic* anchor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.f)), *shape);
		gScene->addActor(*anchor);
		baseJoint = PxFixedJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.f)), base, PxTransform(PxVec3(0.0f)));
		PX_UNUSED(baseJoint);
	}

	//Now create the slider
	gArticulation->setSolverIterationCounts(32);

	anchor = gArticulation->createLink(base, PxTransform(PxVec3(0.f, -distance, 0.f)));
	anchor->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*anchor, 100.f);

	//Set up the drive joint...
	gDriveJoint = static_cast<PxArticulationJointReducedCoordinate*>(anchor->getInboundJoint());
	gDriveJoint->setJointType(PxArticulationJointType::ePRISMATIC);
	gDriveJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);
	float stiffness	= 100.f;
	float damping	= 4.f;
	float forceLimit= PX_MAX_F32;
	gDriveJoint->setDrive(PxArticulationAxis::eX, stiffness, damping, forceLimit);

	gDriveJoint->setParentPose(PxTransform(PxVec3(0.f, -distance, 0.f)));
	gDriveJoint->setChildPose(PxTransform(PxVec3(0.f)));
}

void createRope()
{
	lastLink = anchor;
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(0.1f), *gMaterial);
	const PxVec3 initPos(0.0f, -2*distance, 0.0f);
	PxVec3 pos = initPos;
	// Create rope
	for(PxU32 i=0;i<nLinks;i++)
	{
		PxArticulationLink* link = gArticulation->createLink(lastLink, PxTransform(pos));

		link->attachShape(*shape);
		if(i<nLinks-1){
			PxRigidBodyExt::setMassAndUpdateInertia(*link, mass);
		} else {
			PxRigidBodyExt::setMassAndUpdateInertia(*link, mass/2);
		}


		PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
		joint->setJointType(PxArticulationJointType::eSPHERICAL);
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		joint->setParentPose(PxTransform(PxVec3( 0.0f)));
		joint->setChildPose(PxTransform(PxVec3(  0.0f, distance, 0.0f)));
		pos.y -= distance;

		lastLink = link;
	}
}

void initPhysics(bool /*interactive*/)
{
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

	//sceneDesc.solverType = PxSolverType::eTGS;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);

	gArticulation = gPhysics->createArticulationReducedCoordinate();

	createAttachment();

	initFile();
	createRope();
	gScene->addArticulation(*gArticulation);
}

void stepPhysics(bool /*interactive*/)
{
	static float	t			= 0;
	static const time_t time_start = time(NULL);
	time_t time_current = time(NULL);
	double time_running = difftime(time_current,time_start);
	if(time_wait - time_running >= 0){
		std::cout << "Countdown: " << time_wait - time_running << "\r";
	}
	if(time_running >= time_wait && t <= (float)time_record){
		if(gArticulation->isSleeping()) gArticulation->wakeUp();
		float			F			= amplitude*sin(freq*t);			// [-]
		PxReal			driveValue	= F;
		driveValue	= F;
		gDriveJoint->setDriveTarget(PxArticulationAxis::eX, driveValue);
		PxVec3 basePos, endPos;
		getPositions(&basePos, &endPos);
		logPositions(t, &basePos, &endPos);
		t += dt;
	}
	if(time_running - time_wait > time_record){
		std::cout << "Finished recording\r";
	}
	gScene->simulate(dt);
	gScene->fetchResults(true);

	// Apply drag to tether elements
	//applyDrag();
}
	
void cleanupPhysics(bool /*interactive*/)
{
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
