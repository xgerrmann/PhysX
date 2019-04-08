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

#include <chrono>

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

PxU32 subStepCount						= 20;
const float slowDown					= 1.0f*subStepCount;
const float dt							= 1.0f/60.0f/slowDown;
const int nLinks						= 20;
const float gravity						= 9.81f;
const float distance					= 1.0f;
const float characteristicMass			= 0.013f;	// % [kg/m]
const float mass 						= characteristicMass*distance;	// % [kg]
const float radius						= 0.1f;

float maxReelingVelocity				= 100;				// [m/s]
float dReelingVelocity					= 10.0f/slowDown*subStepCount;	// [m/s]
float reelingVelocity					= 0.0f;				// [m/s]

enum reelingDirection {reelIn = 0, reelOut = 1, None = 2};

enum reelingDirection currentReelingDirection = None;

PxRigidStatic* anchor					= NULL;

PxSphericalJoint* anchorJoint			= NULL;

PxRigidDynamic* box						= NULL;
PxFixedJoint* boxJoint					= NULL;

class Tether {
	public:
		PxArticulationLink* getStartLink();
		PxArticulationLink* getEndLink();
		void createRope();
		void addLink(PxVec3 pos);
		void removeLink();
		int getNbElements();
		Tether(int nLinks);
		void applyDrag();
	private:
		PxArticulationLink* endLink   = NULL;
		PxArticulationLink* startLink = NULL;

		int nbElements = 0;

		static const int maxNbElements = 64;

		PxArticulationLink* tetherElements[maxNbElements];
};

int Tether::getNbElements(){
	return nbElements;
}

PxArticulationLink* Tether::getStartLink(){
	return startLink;
}

PxArticulationLink* Tether::getEndLink(){
	return endLink;
}

void Tether::removeLink()
{
	if (startLink != nullptr)
	{
		// Release, then remove pointer to element.
		tetherElements[nbElements-1]->release();
		tetherElements[nbElements-1] = NULL;
		nbElements -= 1;
		startLink = tetherElements[nbElements-1];
	}
}

Tether* tether;

void Tether::applyDrag(){
	const PxU32 startIndex	= 0;
	PxArticulationLink* links[maxNbElements];
	gArticulation->getLinks(links, nbElements, startIndex);
	static float CD = 0.2f;
	for(int ii = 0; ii<nbElements; ii++){
		PxVec3 vel = links[ii]->getLinearVelocity();
		PxVec3 drag = vel*CD;
		links[ii]->addForce(-drag);
	}
}

void Tether::addLink(PxVec3 pos){
	// Number of elements may not exceed the set maximum
	if(nbElements >= maxNbElements){
		return;
	}
	PxArticulationLink* newLink = gArticulation->createLink(startLink, PxTransform(pos));

	PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
	newLink->attachShape(*sphereShape);
	PxRigidBodyExt::setMassAndUpdateInertia(*newLink, mass);

	PxArticulationJointBase* joint = newLink->getInboundJoint();
	if(joint)	// Will be null for root link
	{
		joint->setParentPose(PxTransform(PxVec3(0.0f)));
		joint->setChildPose(PxTransform(PxVec3(0.0f, -distance, 0.0f)));
	}

	// Update pointers and counter
	startLink = newLink;
	tetherElements[nbElements] = newLink;
	nbElements += 1;

	// Give name to tether elements
	char name[20];
	if(nbElements == 1){
		endLink = newLink;
		sprintf(name,"EndLink");
		newLink->setName(name);
	} else {
		sprintf(name,"Link_%03i",nbElements-1);
		newLink->setName(name);
	}
	std::cout << "Created link: " << newLink->getName() << std::endl;
}

Tether::Tether(int nLinks){
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gArticulation = gPhysics->createArticulation();

	// Stabilization can create artefacts on jointed objects so we just disable it
	gArticulation->setStabilizationThreshold(0.0f);

	gArticulation->setMaxProjectionIterations(16);
	gArticulation->setSeparationTolerance(0.001f);

	PxVec3 pos(0.0f,-nLinks*distance,0.0f);

	// Create rope
	for(int i=0;i<nLinks;i++)
	{
		addLink(pos);
		pos.y += distance;
	}
}

void createAttachment(){
	// Attach articulation to anchor
	PxShape* anchorShape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
	anchor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f)), *anchorShape);
	gScene->addActor(*anchor);
	anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), tether->getStartLink(), PxTransform(PxVec3(0.0f,distance,0.0f)));
}

void attachBox(){
// Attach large & heavy box at the end of the rope

	const float boxMass = 500.0f;
	const float boxHalfSize = 1.0f; // Half the width, height and depth
	const float boxDensity = boxMass/std::pow(2.f*boxHalfSize,3.f);

	PxArticulationLink* tetherLink = tether->getEndLink();
	PxVec3 firstLinkPos = tetherLink->getGlobalPose().p;

	PxVec3 boxPos	= firstLinkPos;
	boxPos.y		-= 2*boxHalfSize;

	PxShape* boxShape = gPhysics->createShape(PxBoxGeometry(PxVec3(boxHalfSize)), *gMaterial);
	box = PxCreateDynamic(*gPhysics, PxTransform(boxPos), *boxShape, boxDensity);
	gScene->addActor(*box);
	boxJoint = PxFixedJointCreate(*gPhysics, box, PxTransform(PxVec3(0.0f,2*distance,0.0f)), tetherLink, PxTransform(PxVec3(0.0f)));
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
	sceneDesc.solverType		= PxSolverType::eTGS;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	tether = new Tether(nLinks);
	createAttachment();
	attachBox();


	gScene->addArticulation(*gArticulation);
}

void stepPhysics(bool /*interactive*/)
{
	static std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> t_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	static float t_elapsed_f = 0.0f;
	std::chrono::duration<double> t_elapsed_ch =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-t_start);
	std::cout << "Chrono: " << t_elapsed_ch.count() << ", simulation: " << t_elapsed_f << "dt: " << dt << std::endl;
	for(PxU32 i=0; i < subStepCount; i++){
		t_elapsed_f += dt;
		gScene->simulate(dt);
		gScene->fetchResults(true);
		// Apply drag to tether elements
		tether->applyDrag();
	}
	// Change anchor position
	static float elongation = distance;

	if(currentReelingDirection == reelIn){
		elongation -= reelingVelocity*dt;
	} else if(currentReelingDirection == reelOut) {
		elongation += reelingVelocity*dt;
	}
	
	if(reelingVelocity != 0 && anchorJoint != nullptr){
		anchorJoint->release();
		anchorJoint = NULL;
		PxArticulationLink* startLink = tether->getStartLink();
		anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f,elongation,0.0f)));
	} else if(anchorJoint == nullptr){
		std::cout << "debug loc 1" << std::endl;
	}

	bool isSleeping = gArticulation->isSleeping();
	if(isSleeping){
		gArticulation->wakeUp();
	}

	// Update tether (add/remove links)
	// Add +/ delete links
	int nbElements = tether->getNbElements();
	if (currentReelingDirection == reelIn && elongation < distance/2 && nbElements > 1)
	{
		// Remove element closest to anchor
		tether->removeLink();
		PxArticulationLink* startLink = tether->getStartLink();
		// Attach link to anchor
		elongation += distance;
		anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f,elongation,0.0f)));
	} else if(currentReelingDirection == reelOut && elongation > distance*2 && nbElements < 64) {

		PxArticulationLink* oldLink = tether->getStartLink();
		PxVec3 oldLinkPosition = oldLink->getGlobalPose().p;
		PxVec3 oldLinkLinearVelocity = oldLink->getLinearVelocity();
		PxVec3 oldLinkAngularVelocity = oldLink->getAngularVelocity();
		PxVec3 newLinkPosition = oldLinkPosition;
		PxVec3 newLinkUnitVector = newLinkPosition;
		newLinkUnitVector.normalize();
		newLinkPosition -= newLinkUnitVector*distance;

		// Remove element closest to anchor
		tether->addLink(newLinkPosition);
		PxArticulationLink* startLink = tether->getStartLink();
		// Attach link to anchor
		anchorJoint->release();
		anchorJoint = NULL;
		elongation -= distance;
		anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f,elongation,0.0f)));
		// Set angular velocity
		startLink->setAngularVelocity(oldLinkAngularVelocity);
		// Set linear velocity
		PxVec3 startLinkLinearVelocity = oldLinkLinearVelocity*elongation/oldLinkPosition.magnitude();
		startLink->setLinearVelocity(-startLinkLinearVelocity);
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
		reelingVelocity = std::max(0.f,reelingVelocity-dReelingVelocity);
	} else if(key=='k') {
		reelingVelocity = std::min(maxReelingVelocity,reelingVelocity+dReelingVelocity);
	} else if(key=='m') {
		float magnitude = 100.0f;
		bool autowake = true;
		box->addForce(PxVec3(magnitude, 0.0f,0.0f), PxForceMode::eIMPULSE,autowake);
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
