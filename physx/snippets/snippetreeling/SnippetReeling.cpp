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
#include <stdio.h>
#include <ctype.h>
#include <vector>

#include "PxPhysicsAPI.h"

#include <chrono>

#include <fstream>

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

PxArticulationReducedCoordinate*	gArticulation = NULL;

PxU32 subStepCount						= 20.0f;
const float slowDown					= 1.0f*subStepCount;
const float dt							= 1.0f/60.0f/slowDown;
const int nLinks						= 20;
const float gravity						= 9.81f;
const float elementLength				= 1.0f;
const float characteristicMass			= 0.013f;	// % [kg/m]
const float radius						= 0.1f;

float maxReelingVelocity				= 100;				// [m/s]
float dReelingVelocity					= 10.0f/slowDown;	// [m/s]
float reelingVelocity					= 0.0f;				// [m/s]

enum reelingDirection {reelIn = 0, reelOut = 1, None = 2};

enum reelingDirection currentReelingDirection = None;

PxRigidStatic* anchor					= NULL;

PxSphericalJoint* anchorJoint			= NULL;

PxRigidDynamic* box						= NULL;
PxFixedJoint* boxJoint					= NULL;

PxArticulationJointReducedCoordinate* driveJoint			= NULL;

const char* fpath = "/home/xander/Google Drive/Thesis/src/reeling_analysis/data/data";
std::fstream file;


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
		PxArticulationLink* getLink(int linkIndex);
	private:
		PxArticulationLink* endLink   = NULL;
		PxArticulationLink* startLink = NULL;

		int nbElements = 0;

		static const int maxNbElements = 64;

		PxArticulationLink* tetherElements[maxNbElements];
};

PxArticulationLink* Tether::getLink(int linkIndex){
	PxArticulationLink* links[1];
	int nLinks = 1;
	gArticulation->getLinks(links, nLinks, linkIndex);
	return links[0];
}

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
	static float CD = 0.05f;
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
	// Update mass of current start element
	// Since it is now an intermediate link, its mass must equal the mass of one complete tether element
	if(startLink != nullptr){
		float startLinkMass = elementLength*characteristicMass;
		PxRigidBodyExt::setMassAndUpdateInertia(*startLink, startLinkMass);
	}

	PxArticulationLink* newLink = gArticulation->createLink(startLink, PxTransform(pos));

	PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);
	newLink->attachShape(*sphereShape);

	// Each link is initialized with half the mass of a tether element.
	float newLinkMass = 0.5f*characteristicMass*elementLength;	// % [kg]
	PxRigidBodyExt::setMassAndUpdateInertia(*newLink, newLinkMass);

	PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(newLink->getInboundJoint());
	if(joint)	// Will be null for root link
	{
		// Joint must be located in child link. The child is the new link.
		joint->setParentPose(PxTransform(PxVec3(0.0f, elementLength, 0.0f)));
		joint->setChildPose(PxTransform(PxVec3(0.0f)));

		// The joint type must be specified for reducedCoordinateArticulations
		joint->setJointType(PxArticulationJointType::eSPHERICAL);

		// Allowed motions must be specified
		joint->setMotion(PxArticulationAxis::eSWING1 , PxArticulationMotion::eFREE);
		joint->setMotion(PxArticulationAxis::eSWING2 , PxArticulationMotion::eFREE);
		joint->setMotion(PxArticulationAxis::eTWIST , PxArticulationMotion::eFREE);
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

	// TODO: set joint type and see if velocity is still correct
}

Tether::Tether(int nLinks){
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	//gArticulation = gPhysics->createArticulation();
	// This is wrongly described in the documentation as:
	// gArticulation = gPhysics->createReducedCoordinateArticulation();
	gArticulation = gPhysics->createArticulationReducedCoordinate();

	//// Stabilization can create artefacts on jointed objects so we just disable it
	//gArticulation->setStabilizationThreshold(0.0f);

	//gArticulation->setMaxProjectionIterations(16);
	//gArticulation->setSeparationTolerance(0.001f);

	PxVec3 pos(0.0f,-(nLinks-1)*elementLength,0.0f);

	// Create rope
	for(int i=0;i<nLinks;i++)
	{
		addLink(pos);
		pos.y += elementLength;
	}
	std::cout << "nbElements: " << nbElements << std::endl;
}

void createAttachment(){
	// Attach articulation to anchor
	PxShape* anchorShape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
	anchor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f)), *anchorShape);
	gScene->addActor(*anchor);
	anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), tether->getStartLink(), PxTransform(PxVec3(0.0f)));
}

void attachBox(){
// Attach large & heavy box at the end of the rope

	const float boxMass = 10.0f;
	const float boxHalfSize = 1.0f; // Half the width, height and depth
	const float boxDensity = boxMass/std::pow(2.f*boxHalfSize,3.f);

	PxArticulationLink* tetherLink = tether->getEndLink();
	PxVec3 firstLinkPos = tetherLink->getGlobalPose().p;

	PxVec3 boxPos	= firstLinkPos;
	boxPos.y		-= 2*boxHalfSize;

	PxShape* boxShape = gPhysics->createShape(PxBoxGeometry(PxVec3(boxHalfSize)), *gMaterial);
	box = PxCreateDynamic(*gPhysics, PxTransform(boxPos), *boxShape, boxDensity);
	gScene->addActor(*box);
	boxJoint = PxFixedJointCreate(*gPhysics, box, PxTransform(PxVec3(0.0f,2*elementLength,0.0f)), tetherLink, PxTransform(PxVec3(0.0f)));
}

void initFile(){
	// Save data
	std::cout << fpath << std::endl;
	std::ofstream file;
	file.open(fpath, std::ios_base::out);
	const static int length_head = 4;
	file << "Length_head: "			<<  length_head << std::endl;
	file << "Timestep: "			<<  dt << std::endl;
	file << "CharacteristicMass: "	<<  characteristicMass << std::endl;
	file << "ElementLength: "		<<  elementLength << std::endl;

	// CSV header
	file << "time," << "nElements,"<< "force_x," << "force_y," << "force_z," << "link_pos," << "link_vel_x," << "link_vel_y," << "link_vel_z" << std::endl;
	file.close();
}

void logForce(float t){
	//PxArticulationLink* sLink = tether->getStartLink();
	//PxArticulationJoint* sJoint = static_cast<PxArticulationJoint*>(sLink->getInboundJoint());
	//PxConstraint* anchorConstraint = anchorJoint->getConstraint();
	//PxConstraint* sConstraint = sJoint->getConstraint();
	//sConstraint->getForce(force, moment);
	PxVec3 force, moment;
	if(anchorJoint != NULL){
		anchorJoint->getConstraint()->getForce(force, moment);
	} else {
		printf("Anchorjoint pointer is NULL.\r\n");
		force = PxVec3(0.0f);
		moment= PxVec3(0.0f);
	}
	int nbElements = tether->getNbElements();

	PxArticulationLink* link = tether->getEndLink();
	//PxArticulationLink* link = tether->getStartLink();
	PxVec3 linkPosition, linkVelocity;
	if(link != nullptr){
		linkPosition = link->getGlobalPose().p;
		linkVelocity = link->getLinearVelocity();
	} else {
		linkPosition = PxVec3(0.0f);
		linkVelocity = PxVec3(0.0f);
	}

	file.open(fpath, std::ios_base::app);
	// Save data
	file << t << "," << nbElements << "," << force.x << "," << force.y << "," << force.z << ","<< linkPosition.y << "," << linkVelocity.x << "," << linkVelocity.y << "," << linkVelocity.z << std::endl;
	file.close();
}

void sphericalJointToPrismatic(PxArticulationJointBase* joint){
	std::cout << "Set joint to prismatic" << std::endl;
	PxArticulationJointReducedCoordinate* reducedCoordinateJoint = static_cast<PxArticulationJointReducedCoordinate*>(joint);
	if(reducedCoordinateJoint){
		std::cout << "Set Joint to Drive" << std::endl;
		// The joint type must be specified for reducedCoordinateArticulations
		reducedCoordinateJoint->setJointType(PxArticulationJointType::ePRISMATIC);
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eY , PxArticulationMotion::eFREE);
		// Disallow motions
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eSWING1 , PxArticulationMotion::eLOCKED);
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eSWING2 , PxArticulationMotion::eLOCKED);
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eTWIST ,  PxArticulationMotion::eLOCKED);
		
		PxReal stiffness = 1000000.0f;
		PxReal damping = 100.0f;
		PxReal forceLimit = PX_MAX_F32;
		reducedCoordinateJoint->setDrive(PxArticulationAxis::eY, stiffness, damping, forceLimit);
		driveJoint = reducedCoordinateJoint;
	}
}

void prismaticJointToSpherical(PxArticulationJointBase* joint){
	std::cout << "Set joint to spherical" << std::endl;
	PxArticulationJointReducedCoordinate* reducedCoordinateJoint = static_cast<PxArticulationJointReducedCoordinate*>(joint);
	if(reducedCoordinateJoint){
		// The joint type must be specified for reducedCoordinateArticulations
		reducedCoordinateJoint->setJointType(PxArticulationJointType::eSPHERICAL);
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eY , PxArticulationMotion::eLOCKED);
		// Disallow motions
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eSWING1 , PxArticulationMotion::eFREE);
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eSWING2 , PxArticulationMotion::eFREE);
		reducedCoordinateJoint->setMotion(PxArticulationAxis::eTWIST ,  PxArticulationMotion::eFREE);
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
	sceneDesc.solverType		= PxSolverType::eTGS;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	initFile();
	tether = new Tether(nLinks);
	createAttachment();
	attachBox();

	// Make sure the root links is positioned exactly at the origin
	//gArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);

	//// Change the joint type of the first articulation
	PxArticulationLink* link = tether->getStartLink();
	sphericalJointToPrismatic( link->getInboundJoint() );

	gScene->addArticulation(*gArticulation);
}


void stepPhysics(bool /*interactive*/)
{
//	static std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> t_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	static bool added = false;
	static float t_elapsed_f = 0.0f;
	static float distance = elementLength;//tether->getStartLink()->getGlobalPose().p.normalize();
//	std::chrono::duration<double> t_elapsed_ch =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-t_start);
//	std::cout << "Chrono: " << t_elapsed_ch.count() << ", simulation: " << t_elapsed_f << "dt: " << dt << std::endl;
	//while(added){}
	for(PxU32 i=0; i < subStepCount; i++){
		//float distance = tether->getStartLink()->getGlobalPose().p.normalize();
		//logForce(t_elapsed_f);
		t_elapsed_f += dt;
		gScene->simulate(dt);
		gScene->fetchResults(true);
		// Apply drag to tether elements
		//tether->applyDrag();
		// Apply force for tether force check
		bool autowake = true;
		PxArticulationLink* lastLink = tether->getEndLink();
		lastLink->addForce(PxVec3(0.0f,-100.0f,0.0f), PxForceMode::eFORCE,autowake);

		// Get constraint force
		logForce(t_elapsed_f);

		// Determine reeling velocity and update distance
		distance -= reelingVelocity*dt;
		if(reelingVelocity ==0){
			currentReelingDirection = None;
		} else if(reelingVelocity > 0 ){
			currentReelingDirection = reelIn;
		} else if(reelingVelocity < 0 ){
			currentReelingDirection = reelOut;
		}

		int nbElements = tether->getNbElements();
		if (currentReelingDirection == reelIn && distance < elementLength/2.0f && nbElements > 1){
		// Reelin + remove element
			// Remove element closest to anchor
			printf("Remove link\r\n");
			tether->removeLink();
			// Relocate startLink
			printf("Relocate\r\n");
			PxArticulationLink* startLink = tether->getStartLink();
			startLink->setGlobalPose(PxTransform(PxVec3(0.0f)));
			// Connect startLink to anchor
			printf("Connect to anchor\r\n");
			anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f)));
			// Change connection to prismatic
			printf("Change joint to prismatic\r\n");
			PxArticulationLink* link = tether->getStartLink();
			sphericalJointToPrismatic( link->getInboundJoint() );
			// Update distance
			distance += elementLength;
			// Change joint pose
			PxArticulationJointReducedCoordinate* reducedCoordinateJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
			reducedCoordinateJoint->setParentPose(PxTransform(PxVec3(0.0f, elementLength, 0.0f)));
			reducedCoordinateJoint->setChildPose(PxTransform(PxVec3(0.0f)));

		//	//std::cout << "distance: " << distance << std::endl;
		//	//std::cout << "pos:      " << startLink->getGlobalPose().p.normalize() << std::endl;
		//	//std::cout << "veldist:  " << reelingVelocity*dt << std::endl;
		//	//std::cout << "corrected:" << startLink->getGlobalPose().p.normalize() - reelingVelocity*dt<< std::endl;
		//	//std::cout << "error:    " << distance - (startLink->getGlobalPose().p.normalize() - reelingVelocity*dt) << std::endl;
		} else if(currentReelingDirection == reelOut && distance > elementLength*2 && nbElements < 64) {
		// Reelout + add element
			// Update distance
			printf("Distance: %f",(double)distance);
			distance -= elementLength;
			printf("Distance: %f",(double)distance);
			// Remove first link
			tether->removeLink();
			// Add 2 new links
			PxVec3 pos = tether->getStartLink()->getGlobalPose().p;
			pos += PxVec3(0.0f,elementLength,0.0f);
			tether->addLink(pos);
			tether->addLink(PxVec3(0.0f));
			PxArticulationLink* startLink = tether->getStartLink();
			// Change joint pose
			PxArticulationJointReducedCoordinate* reducedCoordinateJoint = static_cast<PxArticulationJointReducedCoordinate*>(startLink->getInboundJoint());
			reducedCoordinateJoint->setParentPose(PxTransform(PxVec3(0.0f, distance, 0.0f)));
			reducedCoordinateJoint->setChildPose(PxTransform(PxVec3(0.0f)));
			// Attach new link to anchor
			anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), tether->getStartLink(), PxTransform(PxVec3(0.0f)));
			// Change connection to prismatic
			printf("Change joint to prismatic\r\n");
			PxArticulationLink* link = tether->getStartLink();
			sphericalJointToPrismatic( link->getInboundJoint() );
			
			//// Remove joint between startLink and anchor
			//anchorJoint->release();
			//anchorJoint = NULL;
			//// Relocate startLink
			//printf("Relocate\r\n");
			//PxArticulationLink* startLink = tether->getStartLink();
			//startLink->setGlobalPose(PxTransform(PxVec3(0.0f,-distance,0.0f)));
			//// Change joint type from prismatic to spherical
			//PxArticulationLink* link = tether->getStartLink();
			//prismaticJointToSpherical(link->getInboundJoint());
			//// Change joint pose
			//PxArticulationJointReducedCoordinate* reducedCoordinateJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
			//reducedCoordinateJoint->setParentPose(PxTransform(PxVec3(0.0f, elementLength, 0.0f)));
			//reducedCoordinateJoint->setChildPose(PxTransform(PxVec3(0.0f)));
			//// Add link
			//tether->addLink(PxVec3(0.0f));
			//// Change link type from spherical to prismatic
			//link = tether->getStartLink();
			//sphericalJointToPrismatic(link->getInboundJoint());
			//// Attach new link to anchor
			//anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f)));

			added = true;
			// Add new link

		//	PxArticulationLink* oldLink		= tether->getStartLink();
		//	PxVec3 oldLinkPos				= oldLink->getGlobalPose().p;
		//	float oldLinkDist				= oldLinkPos.magnitude();
		//	PxVec3 unitDirectionVec			= oldLinkPos.getNormalized();
		//	PxVec3 oldLinkLinearVel			= oldLink->getLinearVelocity();
		//	PxVec3 oldLinkAngularVel		= oldLink->getAngularVelocity();
		//	PxVec3 oldLinkRadialVel			= oldLinkLinearVel.dot(unitDirectionVec)*unitDirectionVec;
		//	PxVec3 oldLinkTangentialVel		= oldLinkLinearVel - oldLinkRadialVel;

		//	PxVec3 newLinkPos			= oldLinkPos - unitDirectionVec*elementLength;
		//	PxVec3 newLinkLinearVel			= oldLinkRadialVel + (distance/oldLinkDist)*oldLinkTangentialVel;
		//	PxVec3 newLinkAngularVel		= oldLinkAngularVel;

		//	printf("Distance: %5.4f\n",(double)distance);
		//	printf("Old: px: %5.4f, py: %5.4f, pz: %5.4f\n",(double)oldLinkPos.x,(double)oldLinkPos.y,(double)oldLinkPos.z);
		//	printf("Old: vx: %5.4f, vy: %5.4f, vz: %5.4f\n",(double)oldLinkLinearVel.x,(double)oldLinkLinearVel.y,(double)oldLinkLinearVel.z);
		//	printf("Old: vr: %5.4f, vt: %5.4f\n",(double)oldLinkRadialVel.normalize(),(double)oldLinkTangentialVel.normalize());
		//	printf("New: vx: %5.4f, vy: %5.4f, vz: %5.4f\n",(double)newLinkLinearVel.x,(double)newLinkLinearVel.y,(double)newLinkLinearVel.z);
		//	printf("New: px: %5.4f, py: %5.4f, pz: %5.4f\n",(double)newLinkPos.x,(double)newLinkPos.y,(double)newLinkPos.z);

		//	// Remove element closest to anchor
		//	tether->addLink(newLinkPos);
		//	PxArticulationLink* startLink = tether->getStartLink();
		//	// Attach link to anchor
		//	anchorJoint->release();
		//	anchorJoint = NULL;
		//	anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f,distance,0.0f)));
		//	// Set linear and angular velocity
		//	startLink->setLinearVelocity(newLinkLinearVel);
		//	startLink->setAngularVelocity(newLinkAngularVel);
		//	added = true;
		//} else if(currentReelingDirection != None && anchorJoint != nullptr){
		//// Change distance without adding or removing elements
		//	anchorJoint->release();
		//	anchorJoint = NULL;
		//	PxArticulationLink* startLink = tether->getStartLink();
		//	anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), startLink, PxTransform(PxVec3(0.0f,distance,0.0f)));
		//	// Update mass of start link
		//	// Start link mass is the weight of the whole tether spanning between the anchor and the first link and half the weight of the following tether element.
		//	float startElementMass = (abs(distance)+0.5f*elementLength)*characteristicMass;
		//	PxRigidBodyExt::setMassAndUpdateInertia(*startLink, startElementMass);
		} else{
			//if(added){getchar();}
			std::cout << "Set drive target" << std::endl;
			driveJoint->setDriveTarget(PxArticulationAxis::eY, distance);
			driveJoint->setDriveVelocity(PxArticulationAxis::eY, reelingVelocity);
			std::cout << reelingVelocity << std::endl;

			bool isSleeping = gArticulation->isSleeping();
			if(isSleeping){
				gArticulation->wakeUp();
			}
		}
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
	if(key=='j') {
		reelingVelocity = std::max(-maxReelingVelocity,reelingVelocity-dReelingVelocity);
	} else if(key=='k') {
		reelingVelocity = std::min(maxReelingVelocity,reelingVelocity+dReelingVelocity);
	} else if(key=='m') {
		float magnitude = 1.0f;
		bool autowake = true;
		//box->addForce(PxVec3(magnitude, 0.0f,0.0f), PxForceMode::eIMPULSE,autowake);
		// Apply force for tether force check
		tether->getLink(1)->addForce(PxVec3(magnitude, 0.0f,0.0f), PxForceMode::eIMPULSE,autowake);
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
