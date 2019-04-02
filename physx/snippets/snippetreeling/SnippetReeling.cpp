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
const int nLinks						= 20;
const float gravity						= 9.81f;
const float distance					= 1.0f;
const float mass 						= 1.0f;
const float radius						= 0.1f;

float reelingVelocity					= 0.01f;

int stepNumber = 0;

enum reelingDirection {reelIn = 0, reelOut = 1, None = 2};

#define REELIN 0
#if REELIN
enum reelingDirection currentReelingDirection = reelIn;
#else
enum reelingDirection currentReelingDirection = reelOut;
#endif

PxArticulationLink* firstLink			= NULL;
PxRigidStatic* anchor					= NULL;

PxSphericalJoint* anchorJoint			= NULL;

PxArticulationLink* GetTailLink()
{
	PxU32 NumLinks = gArticulation->getNbLinks();
	if (NumLinks <= 0)
	{
		return nullptr;
	}

	PxArticulationLink* LinkBuf[1];
	gArticulation->getLinks(LinkBuf, 1, NumLinks - 1);
	PxArticulationLink* TailLink = LinkBuf[0];
	return TailLink;
}

void RemoveLeafLink()
{
	PxArticulationLink* TailLink = GetTailLink();
	if (TailLink != nullptr)
	{
		TailLink->release();
		// Update lastLink
		lastLink = GetTailLink();
	}
}

//void applyDrag(){
//	const PxU32 startIndex	= 0;
//	PxArticulationLink* links[nLinks];
//	gArticulation->getLinks(links, nLinks, startIndex);
//	static float CD = 0.2f;
//	for(int ii = 0; ii<nLinks; ii++){
//		PxVec3 vel = links[ii]->getLinearVelocity();
//		PxVec3 drag = vel*CD;
//		links[ii]->addForce(-drag);
//	}
//}

void createAttachment(){
	// Attach articulation to static world
	PxShape* anchorShape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
	anchor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f)), *anchorShape);
	gScene->addActor(*anchor);
	anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), lastLink, PxTransform(PxVec3(0.0f,distance,0.0f)));
}

PxArticulationLink* addLink(PxArticulationLink* parent, PxVec3* pos, PxShape* shape){
	PxArticulationLink* link = gArticulation->createLink(parent, PxTransform(*pos));

	link->attachShape(*shape);
	PxRigidBodyExt::setMassAndUpdateInertia(*link, mass);

	PxArticulationJointBase* joint = link->getInboundJoint();
	if(joint)	// Will be null for root link
	{
		joint->setParentPose(PxTransform(PxVec3(0.0f)));
		joint->setChildPose(PxTransform(PxVec3(0.0f, -distance, 0.0f)));
	}
	pos->y += distance;
	lastLink = link;
	return link;
}

void createRope(){

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gArticulation = gPhysics->createArticulation();

	// Stabilization can create artefacts on jointed objects so we just disable it
	gArticulation->setStabilizationThreshold(0.0f);

	gArticulation->setMaxProjectionIterations(16);
	gArticulation->setSeparationTolerance(0.001f);


	const PxVec3 initPos(0.0f,-nLinks*distance,0.0f);
	PxVec3 pos = initPos;
	PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
	PxArticulationLink* parent = NULL;

	// Create rope
	for(PxU32 i=0;i<nLinks;i++)
	{
		parent = addLink(parent, &pos, sphereShape);
		if(!firstLink)
			firstLink = parent;
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
	gScene->simulate(dt);
	gScene->fetchResults(true);
	static float posY = 0;
	if(currentReelingDirection == reelIn){
		posY += reelingVelocity;
	} else {
		posY -= reelingVelocity;
	}
	bool autowake = true;
	anchor->setGlobalPose(PxTransform(PxVec3(0.0f,posY,0.0f)),autowake);

	// Apply drag to tether elements
	//applyDrag();

	// Add +/ delete links
	PxU32 NumLinks = gArticulation->getNbLinks();
	if (currentReelingDirection == reelIn && posY >= 0 && NumLinks > 1)
	{
		// Remove element closest to anchor
		RemoveLeafLink();
		// Update anchor location
		PxTransform pose = lastLink->getGlobalPose();
		posY = pose.p.y + distance;
		anchor->setGlobalPose(PxTransform(PxVec3(0.0f,posY,0.0f)),autowake);
		// Attach link to anchor
		anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), lastLink, PxTransform(PxVec3(0.0f,distance,0.0f)));
	} else if(currentReelingDirection == reelOut && posY < -1 && NumLinks < 64) {
		PxTransform anchorPose = anchor->getGlobalPose();
		anchorPose.p.y = anchorPose.p.y + distance;
		posY = anchorPose.p.y;
		// Detach anchor
		if(anchorJoint != nullptr){
			anchorJoint->release();
			anchorJoint = NULL;
		}
		// Update anchor location
		anchor->setGlobalPose(anchorPose,autowake);
		// Create new link
		PxArticulationLink* TailLink = GetTailLink();
		PxTransform linkTF = TailLink->getGlobalPose();
		PxVec3 linkPose = linkTF.p;
		linkPose.y = posY+distance;
		PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
		addLink(TailLink, &linkPose, sphereShape);
		// Attach link to anchor
		anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), lastLink, PxTransform(PxVec3(0.0f,distance,0.0f)));
	}
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

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	std::cout << key << std::endl;
	if(key=='i'){
		currentReelingDirection = reelIn;
	} else if(key=='o') {
		currentReelingDirection = reelOut;
	} else if(key=='j') {
		reelingVelocity = std::max(0.f,reelingVelocity-0.01f);
	} else if(key=='k') {
		reelingVelocity = std::min(1.f,reelingVelocity+0.01f);
	}
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
