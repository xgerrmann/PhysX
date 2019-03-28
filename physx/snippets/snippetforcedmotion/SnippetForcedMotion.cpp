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
const int								nbCapsules		= 30;
const float 							radius			= 0.1f;
const float 							distance		= 1.0f;
const float 							mass			= 1.0f;

void applyDrag(){
	const PxU32 n_links		= nbCapsules;
	const PxU32 startIndex	= 0;
	PxArticulationLink* links[n_links];
	gArticulation->getLinks(links, n_links, startIndex);
	static float CD = 0.5f;
	for(PxU32 ii = 0; ii<n_links; ii++){
		PxVec3 vel = links[ii]->getLinearVelocity();
		PxVec3 drag = vel*CD;
		links[ii]->addForce(-drag);
	}
}

void createAttachment()
{
	// Create base...
	PxArticulationLink* base = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.f, 0.f)));

	PxShape* shape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
	// Attach base articulation to static world
	{
		PxRigidStatic* anchor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.f)), *shape);
		gScene->addActor(*anchor);
		baseJoint = PxFixedJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), base, PxTransform(PxVec3(0.0f)));
		PX_UNUSED(baseJoint);
	}

	//Now create the slider
	gArticulation->setSolverIterationCounts(32);

	anchor = gArticulation->createLink(base, PxTransform(PxVec3(0.f, 0.f, 0.f)));
	anchor->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*anchor, 100.f);

	//Set up the drive joint...
	gDriveJoint = static_cast<PxArticulationJointReducedCoordinate*>(anchor->getInboundJoint());
	gDriveJoint->setJointType(PxArticulationJointType::ePRISMATIC);
	gDriveJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);
	float stiffness	= 100.f;
	float damping	= 4.f;
	gDriveJoint->setDrive(PxArticulationAxis::eX, stiffness, damping, PX_MAX_F32);

	gDriveJoint->setParentPose(PxTransform(PxVec3(0.f, -0.2f, 0.f)));
	gDriveJoint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
}

void createRope()
{
	lastLink = anchor;
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
	const PxVec3 initPos(0.0f, -0.2f, 0.0f);
	PxVec3 pos = initPos;
	// Create rope
	for(PxU32 i=0;i<nbCapsules;i++)
	{
		//pos.y += (radius + distance) * 2.0f;
		PxArticulationLink* link = gArticulation->createLink(lastLink, PxTransform(pos));

		link->attachShape(*shape);
		PxRigidBodyExt::setMassAndUpdateInertia(*link, mass);

		PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
		joint->setJointType(PxArticulationJointType::eSPHERICAL);
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		joint->setParentPose(PxTransform(PxVec3( 0.0f, 0.0f, 0.0f)));
		joint->setChildPose(PxTransform(PxVec3(  distance, 0.0f, 0.0f)));
		pos.x += (radius + distance) * 2.0f;

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
	createRope();
	gScene->addArticulation(*gArticulation);
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = 1.0f / 60.f;
	static float t = 0;
	static const float freq			= 2.0f;								// [Hz]
	static const float amplitude	= 1.0f ;								//[m]
	float F							= amplitude*sin(freq*t);			// [-]
	PxReal driveValue = gDriveJoint->getDriveTarget(PxArticulationAxis::eX);

	driveValue = F;
	gDriveJoint->setDriveTarget(PxArticulationAxis::eX, driveValue);

	gScene->simulate(dt);
	gScene->fetchResults(true);
	t += (double) dt;

	// Apply drag to tether elements
	applyDrag();
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
