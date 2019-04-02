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

PxArticulation*			gArticulation	= NULL;

PxArticulationLink*		lastLink		= NULL;

const float dt							= 1.0f/60.0f;
const int nLinks						= 10;
const float gravity						= 9.81f;
const float distance					= 1.0f;
const float mass 						= 1.0f;
const float radius						= 0.1f;

PxArticulationLink* firstLink			= NULL;
PxRigidStatic* anchor					= NULL;
//PxRigidDynamic* anchor					= NULL;

void applyDrag(){
	const PxU32 startIndex	= 0;
	PxArticulationLink* links[nLinks];
	gArticulation->getLinks(links, nLinks, startIndex);
	static float CD = 0.2f;
	for(int ii = 0; ii<nLinks; ii++){
		PxVec3 vel = links[ii]->getLinearVelocity();
		PxVec3 drag = vel*CD;
		links[ii]->addForce(-drag);
	}
}

void createAttachment(){
	// Attach articulation to static world
	PxShape* anchorShape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
	anchor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f)), *anchorShape);
	//PxReal density = 0.1;
	//anchor = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0.0f)), *anchorShape, density);
	gScene->addActor(*anchor);
	PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), firstLink, PxTransform(PxVec3(0.0f)));
	PX_UNUSED(j);
}

void createRope(){

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gArticulation = gPhysics->createArticulation();

	// Stabilization can create artefacts on jointed objects so we just disable it
	gArticulation->setStabilizationThreshold(0.0f);

	gArticulation->setMaxProjectionIterations(16);
	gArticulation->setSeparationTolerance(0.001f);


	const PxVec3 initPos(0.0f);
	PxVec3 pos = initPos;
	PxShape* capsuleShape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
	PxArticulationLink* parent = NULL;

	// Create rope
	for(PxU32 i=0;i<nLinks;i++)
	{
		PxArticulationLink* link = gArticulation->createLink(parent, PxTransform(pos));
		if(!firstLink)
			firstLink = link;

		link->attachShape(*capsuleShape);
		PxRigidBodyExt::setMassAndUpdateInertia(*link, mass);

		PxArticulationJointBase* joint = link->getInboundJoint();
		if(joint)	// Will be null for root link
		{
			joint->setParentPose(PxTransform(PxVec3(0.0f)));
			joint->setChildPose(PxTransform(PxVec3(0.0f, distance, 0.0f)));
		}
		pos.y -= distance;
		parent = link;
	}
	lastLink = parent;
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

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	createRope();
	createAttachment();

	gScene->addArticulation(*gArticulation);
}

void stepPhysics(bool /*interactive*/)
{
	static float tElapsed = 0;
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);

	float posX = sin(2.f*3.1415f*tElapsed);
	std::cout << posX << std::endl;
	bool autowake = true;
	anchor->setGlobalPose(PxTransform(PxVec3(posX,0.0f,0.0f)),autowake);

	// Apply drag to tether elements
	applyDrag();

	tElapsed += dt;
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
