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

#include <cassert>

using namespace physx;

// Forward declarations
class Kite;
class Tether;
Kite* kite;
Tether* tether;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation		= NULL;
PxPhysics*				gPhysics		= NULL;
PxCooking*				gCooking		= NULL;

PxDefaultCpuDispatcher*	gDispatcher		= NULL;
PxScene*				gScene			= NULL;

PxMaterial*				gMaterial		= NULL;

PxPvd*                  gPvd			= NULL;

PxArticulationCache* articulationCache	= NULL;


PxArticulationReducedCoordinate*	gArticulation = NULL;

PxU32 subStepCount						= 20.0f;
const float slowDown					= 1.0f;
const float dt							= 1.0f/60.0f/slowDown/subStepCount;
const int nLinks						= 10;
const float gravity						= 9.81f;
const float elementLength				= 10.0f;
const float characteristicMass			= 0.013f;	// % [kg/m]
const float radius						= 0.2f;

float maxReelingVelocity				= 100;				// [m/s]
float dReelingVelocity					= 10.0f/slowDown/subStepCount;	// [m/s]
float reelingVelocity					= 0.0f;				// [m/s]

enum reelingDirection {reelIn = 0, reelOut = 1, None = 2};

enum reelingDirection currentReelingDirection = None;

PxRigidDynamic* anchor					= nullptr;

PxFixedJoint* anchorFixedJoint			= nullptr;
PxD6Joint* anchorD6Joint				= nullptr;



PxArticulationJointReducedCoordinate* driveJoint			= nullptr;

bool flying								= true;		// [-]
//float windspeed							= 40;		// [m/s]
float liftForce							= 5000;		// [N]
PxVec3 windVelocity(-20.0f,0.0f,0.0f);				// [m/s]

const char* fpath = "/home/xander/Google Drive/Thesis/src/reeling_analysis/data/data";
std::fstream file;

class Kite {
	const float kiteMass	= 50.0f;	// [kg]
	const float kiteWidth	= 2.0f;		// [m]
	const float kiteHeight	= 0.5f;		// [m]
	const float kiteDepth	= 1.0f;		// [m]
	const float CL			= 0.5;		// [-]
	const float CD			= 0.4;		// [-]

	public:
		Kite(PxArticulationLink* link);

		void getJointForce(PxVec3& jointForce, PxVec3& jointMoment);
		void addForce(PxVec3 force);

		void runAerodynamics();

	private:
		PxRigidDynamic*	kiteDynamic	= nullptr;
		PxFixedJoint*	kiteJoint	= nullptr;

		PxConvexMesh*	createMesh();
};

Kite::Kite(PxArticulationLink* link){
	PxVec3 endLinkPos			= link->getGlobalPose().p;

	const float kiteDensity = kiteMass/(kiteWidth*kiteDepth*kiteHeight);
	PxVec3 kitePos	= endLinkPos;
	kitePos.y		+= 2*kiteHeight;

	PxConvexMesh* kiteMesh			= createMesh();
	PxShape* tmpShape	= gPhysics->createShape(PxBoxGeometry(PxVec3(0.5f)), *gMaterial);
	kiteDynamic			= PxCreateDynamic(*gPhysics, PxTransform(kitePos), *tmpShape, kiteDensity);
	PxShape* kiteShape	= PxRigidActorExt::createExclusiveShape(*kiteDynamic,PxConvexMeshGeometry(kiteMesh), *gMaterial);
	kiteDynamic->attachShape(*kiteShape);
	kiteDynamic->detachShape(*tmpShape);
	kiteJoint			= PxFixedJointCreate(*gPhysics, kiteDynamic, PxTransform(PxVec3(0.0f,-2*kiteHeight,0.0f)), link, PxTransform(PxVec3(0.0f)));
	gScene->addActor(*kiteDynamic);
}

PxConvexMesh* Kite::createMesh(){
	//static const PxVec3 convexVerts[] = {PxVec3(0,1,0),PxVec3(1,0,0),PxVec3(-1,0,0),PxVec3(0,0,1),PxVec3(0,0,-1)};
	static const PxVec3 convexVerts[] = {PxVec3(1,0,3),PxVec3(1,0,-3),PxVec3(-1,0,3),PxVec3(-1,0,-3)};

	PxConvexMeshDesc convexDesc;
	convexDesc.points.count     = 4;
	convexDesc.points.stride    = sizeof(PxVec3);
	convexDesc.points.data      = convexVerts;
	convexDesc.flags            = PxConvexFlag::eCOMPUTE_CONVEX;

	PxDefaultMemoryOutputStream buf;
	PxConvexMeshCookingResult::Enum result;
	if(!gCooking->cookConvexMesh(convexDesc, buf, &result)){
		return NULL;
	}
	PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
	PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);

	return convexMesh;
}

void Kite::getJointForce(PxVec3& jointForce, PxVec3& jointMoment){
	if(kiteJoint!=nullptr){
		kiteJoint->getConstraint()->getForce(jointForce, jointMoment);
	} else {
		jointForce = PxVec3(PxZero);
		jointMoment = PxVec3(PxZero);
	}
}

void Kite::addForce(PxVec3 force){
	static const bool autowake = true;
	kiteDynamic->addForce(force, PxForceMode::eFORCE,autowake);
}

void Kite::runAerodynamics(){
	printf("WindSpeed: %5.5f, %5.5f, %5.5f\r\n",(double) windVelocity.x, (double)windVelocity.y, (double)windVelocity.z);
	float rho = 1.225;		// [kg/m^3]
	float kiteArea = 10;	// [m^2]
	float half = 0.5;		// [-]
	PxVec3 kiteSpeed= kiteDynamic->getLinearVelocity();
	printf("KiteSpeed: %5.5f, %5.5f, %5.5f\r\n",(double) kiteSpeed.x, (double)kiteSpeed.y, (double)kiteSpeed.z);
	PxVec3 va		= windVelocity - kiteSpeed;
	float va2		= va.magnitudeSquared();

	PxVec3 ez(0,0,1); // Unit vector in out-of (YX)plane direction, so Z direction
	PxVec3 crossProduct = va.cross(ez);
	PxVec3 lift = half*rho*va2*kiteArea*CL*crossProduct/crossProduct.magnitude();

	PxVec3 drag = half*va2*kiteArea*CD*va.getNormalized();

	addForce(lift);
	addForce(drag);
	printf("LIFT X: %5.5f, Y: %5.5f, Z: %5.5f\r\n",(double)lift.x,(double)lift.y,(double)lift.z);
	printf("DRAG X: %5.5f, Y: %5.5f, Z: %5.5f\r\n",(double)drag.x,(double)drag.y,(double)drag.z);
}


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
	if(linkIndex-1 < 0){
		return NULL;
	}
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


void Tether::applyDrag(){
	const PxU32 startIndex	= 0;
	PxArticulationLink* links[maxNbElements];
	gArticulation->getLinks(links, nbElements, startIndex);
	static float CD = 0.5f;
	for(int ii = 0; ii<nbElements; ii++){
		PxVec3 linkVelocity = links[ii]->getLinearVelocity();
		PxVec3 apparentVelocity = windVelocity - linkVelocity;
		PxVec3 drag = apparentVelocity*CD;
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
		joint->setParentPose(PxTransform(PxVec3(0.0f, -elementLength, 0.0f)));
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

	PxVec3 pos(0.0f,nLinks*elementLength,0.0f);

	// Create rope
	for(int i=0;i<nLinks;i++)
	{
		addLink(pos);
		pos.y -= elementLength;
	}
	std::cout << "nbElements: " << nbElements << std::endl;
}

void createAttachment(float driveTarget){
	// Attach articulation to anchor
	anchorD6Joint = PxD6JointCreate(*gPhysics, tether->getStartLink(), PxTransform(PxVec3(PxZero)), anchor, PxTransform(PxVec3(PxZero)));
	anchorD6Joint->setMotion(PxD6Axis::eX,      PxD6Motion::eLOCKED);
//	anchorD6Joint->setMotion(PxD6Axis::eY,      PxD6Motion::eLOCKED);
	anchorD6Joint->setMotion(PxD6Axis::eY,      PxD6Motion::eFREE); // Driven axis
	anchorD6Joint->setMotion(PxD6Axis::eZ,      PxD6Motion::eLOCKED);
	anchorD6Joint->setMotion(PxD6Axis::eTWIST,  PxD6Motion::eFREE);
	anchorD6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	anchorD6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	// Set drive
	PxReal driveStiffness	= 10000000.0f;
	PxReal driveDamping		= 1000000.f;
	PxReal driveForceLimit	= PX_MAX_REAL; // + PX_MAX_F32
	bool isAcceleration		= false;
	//PxD6JointDriveFlag::eACCELERATION is recommended, however the following leads to increased oscillations:
	//bool isAcceleration		= true;
	PxD6JointDrive anchorDrive(driveStiffness, driveDamping, driveForceLimit, isAcceleration);
	anchorD6Joint->setDrive(PxD6Drive::eY, anchorDrive);

//	PxVec3 zeroVec(PxZero);
	bool autowake = true;
	anchorD6Joint->setDrivePosition(PxTransform(PxVec3(0.0f,-driveTarget,0.0f)),autowake);
//	anchorD6Joint->setDriveVelocity(zeroVec,zeroVec, autowake);
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
	file<< "time,"
		<< "nElements,"
		<< "anchorForce_x,"
		<< "anchorForce_y,"
		<< "anchorForce_z,"
		<< "boxJointForce_x,"
		<< "boxJointForce_y,"
		<< "boxJointForce_z,"
		<< "startLink_pos_x,"
		<< "startLink_pos_y,"
		<< "startLink_pos_z,"
		<< "startLink_vel_x,"
		<< "startLink_vel_y,"
		<< "startLink_vel_z,"
		<< "secondLink_pos_x,"
		<< "secondLink_pos_y,"
		<< "secondLink_pos_z,"
		<< "secondLink_vel_x,"
		<< "secondLink_vel_y,"
		<< "secondLink_vel_z,"
		<< "endLink_pos_x,"
		<< "endLink_pos_y,"
		<< "endLink_pos_z,"
		<< "endLink_vel_x,"
		<< "endLink_vel_y,"
		<< "endLink_vel_z"
		<< std::endl;
	file.close();
}

void logForce(float t){
//	printf("logForce\r\n");

	PxVec3 anchorForce, anchorMoment;
	PxConstraint* anchorConstraint = anchorFixedJoint->getConstraint();
	if(anchorConstraint != NULL){
		anchorConstraint->getForce(anchorForce, anchorMoment);
	} else {
		printf("AnchorConstraint pointer is NULL.\r\n");
		anchorForce = PxVec3(0.0f);
		anchorMoment= PxVec3(0.0f);
	}
//	double Fx = (double) anchorForce.x;
//	double Fy = (double) anchorForce.y;
//	double Fz = (double) anchorForce.z;
//	double Mx = (double) anchorMoment.x;
//	double My = (double) anchorMoment.y;
//	double Mz = (double) anchorMoment.z;
//	printf("Fx: %+05.5f, \t Fy: %+05.5f, \t Fz: %+05.5f, \t Mx: %+05.5f, \t My: %+05.5f, \t Mz: %+05.5f\r\n", Fx, Fy, Fz, Mx, My, Mz);
	int nbElements = tether->getNbElements();

	PxArticulationLink* startLink	= tether->getStartLink();
	PxArticulationLink* secondLink	= NULL;
	if(nbElements-2 >= 0){
		secondLink	= tether->getLink(nbElements-2);
	}
	PxArticulationLink* endLink		= tether->getEndLink();
	PxVec3 boxJointForce, boxJointMoment, startLinkPos, startLinkVel, secondLinkPos, secondLinkVel, endLinkPos, endLinkVel;
	kite->getJointForce(boxJointForce, boxJointMoment);

	if(startLink != nullptr){
		startLinkPos = startLink->getGlobalPose().p;
		startLinkVel = startLink->getLinearVelocity();
	} else {
		startLinkPos = PxVec3(0.0f);
		startLinkVel = PxVec3(0.0f);
	}
	if(secondLink != nullptr){
		secondLinkPos = secondLink->getGlobalPose().p;
		secondLinkVel = secondLink->getLinearVelocity();
	} else {
		secondLinkPos = PxVec3(0.0f);
		secondLinkVel = PxVec3(0.0f);
	}
	if(endLink != nullptr){
		endLinkPos = endLink->getGlobalPose().p;
		endLinkVel = endLink->getLinearVelocity();
	} else {
		endLinkPos = PxVec3(0.0f);
		endLinkVel = PxVec3(0.0f);
	}

	file.open(fpath, std::ios_base::app);
	// Save data
	file<< t << ","
		<< nbElements << ","
		<< anchorForce.x << ","
		<< anchorForce.y << ","
		<< anchorForce.z << ","
		<< boxJointForce.x << ","
		<< boxJointForce.y << ","
		<< boxJointForce.z << ","
		<< startLinkPos.x << ","
		<< startLinkPos.y << ","
		<< startLinkPos.z << ","
		<< startLinkVel.x << ","
		<< startLinkVel.y << ","
		<< startLinkVel.z << ","
		<< secondLinkPos.x << ","
		<< secondLinkPos.y << ","
		<< secondLinkPos.z << ","
		<< secondLinkVel.x << ","
		<< secondLinkVel.y << ","
		<< secondLinkVel.z << ","
		<< endLinkPos.x << ","
		<< endLinkPos.y << ","
		<< endLinkPos.z << ","
		<< endLinkVel.x << ","
		<< endLinkVel.y << ","
		<< endLinkVel.z << ","
		<< std::endl;
	file.close();
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
	assert(gCooking != NULL);

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
	tether	= new Tether(nLinks);
	PxArticulationLink* endLink	= tether->getEndLink();
	kite	= new Kite(endLink);
	gScene->addArticulation(*gArticulation);
	articulationCache= gArticulation->createCache();

	// Create object at origin to visualize the origin's location
	PxShape* anchorShape= gPhysics->createShape(PxSphereGeometry(1.0f), *gMaterial);
	anchor				= PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0.0f)), *anchorShape,10.0f); // Last value is DENSITY, not weight.
	anchorFixedJoint	= PxFixedJointCreate(*gPhysics, anchor, PxTransform(PxVec3(PxZero)), NULL, PxTransform(PxVec3(PxZero)));
	//anchorFixedJoint	= PxFixedJointCreate(*gPhysics, NULL ,PxTransform(PxVec3(PxZero)), anchor, PxTransform(PxVec3(PxZero)));
	gScene->addActor(*anchor);

	createAttachment(elementLength);
}

void printPositions(){
	printf("Print positions\r\n");
	PxArticulationLink* firstLink = tether->getStartLink();
	PxArticulationLink* secondLink= tether->getLink(tether->getNbElements()-2);
	PxVec3 secondLinkVel(PxZero), secondLinkPos(PxZero), firstLinkVel(PxZero), firstLinkPos(PxZero);
	if(firstLink != NULL){
		firstLinkPos = firstLink->getGlobalPose().p;
		firstLinkVel = firstLink->getLinearVelocity();
	}
	if(secondLink != NULL){
		secondLinkPos = secondLink->getGlobalPose().p;
		secondLinkVel = secondLink->getLinearVelocity();
	}
	printf("First  link pos: %3.10f, %3.10f, %3.10f\r\n",(double)firstLinkPos.x, (double)firstLinkPos.y, (double)firstLinkPos.z);
	printf("Second link pos: %3.10f, %3.10f, %3.10f\r\n",(double)secondLinkPos.x,(double)secondLinkPos.y,(double)secondLinkPos.z);
	printf("First  link vel: %3.10f, %3.10f, %3.10f\r\n",(double)firstLinkVel.x, (double)firstLinkVel.y, (double)firstLinkVel.z);
	printf("Second link vel: %3.10f, %3.10f, %3.10f\r\n",(double)secondLinkVel.x,(double)secondLinkVel.y,(double)secondLinkVel.z);
}

void printCache(){
	//gArticulation->releaseCache(*articulationCache);
	//articulationCache= gArticulation->createCache();
	const int maxLinks = 64;
	PxArticulationLink* links[maxLinks];
	PxU32 TotalLinkCount  = gArticulation->getLinks(links, maxLinks, 0);

	std::cout << "Total links: " << TotalLinkCount << std::endl;

	PxU32 dofStarts[maxLinks];
	dofStarts[0] = 0; //We know that the root link does not have a joint

	for(PxU32 i = 1; i < TotalLinkCount; ++i)
	{
		PxU32 llIndex = links[i]->getLinkIndex();
		PxU32 dofs = links[i]->getInboundJointDof();

		dofStarts[llIndex] = dofs;
	}

	PxU32 count = 0;
	for(PxU32 i = 1; i < TotalLinkCount; ++i)
	{
		PxU32 dofs = dofStarts[i];
		dofStarts[i] = count;
		count += dofs;
	}
	gArticulation->copyInternalStateToCache(*articulationCache, PxArticulationCache::eALL);
	// Print position, velocity and acceleration of each DOF
	for(int ii = 0; ii < (int)TotalLinkCount; ii++){
		int nDOF = links[ii]->getInboundJointDof();
		printf("Link: %02d, #DOF: %d\r\n",ii, nDOF);
		for(int jj = 0; jj<(int)nDOF; jj++){
			PxReal pos	= articulationCache->jointPosition[dofStarts[links[ii]->getLinkIndex()]+jj];
			PxReal vel	= articulationCache->jointVelocity[dofStarts[links[ii]->getLinkIndex()]+jj];
			PxReal acc	= articulationCache->jointAcceleration[dofStarts[links[ii]->getLinkIndex()]+jj];
			printf("DOF#: %02d, pos: %05.6f, \t vel: %05.6f, \t acc: %05.6f\r\n",jj,(double)pos,(double)vel,(double)acc);
		}
	}
}

void stepPhysics(bool /*interactive*/)
{
//	static std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> t_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	//static bool added = false;
	static float t_elapsed_f = 0.0f;
	static float driveTarget = elementLength;//tether->getStartLink()->getGlobalPose().p.normalize();
	//printf("Distance: %5.6f\r\n",(double)driveTarget);
//	std::chrono::duration<double> t_elapsed_ch =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-t_start);
//	std::cout << "Chrono: " << t_elapsed_ch.count() << ", simulation: " << t_elapsed_f << "dt: " << dt << std::endl;
	//while(added){}
	for(PxU32 i=0; i < subStepCount; i++){
		if(flying){
			kite->addForce(PxVec3(0.0f,liftForce, 0.0f));
			kite->runAerodynamics();
		}
		logForce(t_elapsed_f);
		//printPositions();
		t_elapsed_f += dt;
		
		gScene->simulate(dt);
		gScene->fetchResults(true);

		// Apply drag to tether elements
//		tether->applyDrag();

		// Determine reeling velocity and update driveTarge
		// Positive reeling velocity means reeling in
		driveTarget -= reelingVelocity*dt;
		if(reelingVelocity ==0){
			currentReelingDirection = None;
		} else if(reelingVelocity > 0 ){
			currentReelingDirection = reelIn;
		} else if(reelingVelocity < 0 ){
			currentReelingDirection = reelOut;
		}
		int nbElements = tether->getNbElements();
		if (currentReelingDirection == reelIn && driveTarget < elementLength/2.0f && nbElements > 1){
		// Reelin + remove element
			// Remove element closest to anchor
			printf("Reelin: Remove link.\r\n");
			tether->removeLink();
			// Update driveTarget
			driveTarget += elementLength;
			// Connect startLink to anchor
			createAttachment(driveTarget);
//			// Change joint pose
			// TODO
		} else if(currentReelingDirection == reelOut && driveTarget > elementLength*1.5f && nbElements < 64) {
			printf("Reelout: Add link.\r\n");
			// Release D6 joint
			PxConstraint* anchorConstraint= anchorD6Joint->getConstraint();
			anchorConstraint->release(); // Anchor destructor also calls the destructor of the related joint
			anchorD6Joint = nullptr; // Remove reference to released joint

			// Create new link
			PxVec3 startLinkPos = tether->getStartLink()->getGlobalPose().p;
			PxVec3 pos = startLinkPos - startLinkPos/(startLinkPos.magnitude())*elementLength;
			tether->addLink(pos);

			// Update drive target
			driveTarget -= elementLength;

			// Create Attachment
			createAttachment(driveTarget);


			// Change joint pose
			// TODO

//			PxArticulationJointReducedCoordinate* reducedCoordinateJoint = static_cast<PxArticulationJointReducedCoordinate*>(startLink->getInboundJoint());
//			reducedCoordinateJoint->setParentPose(PxTransform(PxVec3(0.0f, driveTarget, 0.0f)));
//			reducedCoordinateJoint->setChildPose(PxTransform(PxVec3(0.0f)));
//			// Attach new link to anchor
//			anchorJoint = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), tether->getStartLink(), PxTransform(PxVec3(0.0f)));
//			// Change connection to prismatic
//			printf("Change joint to prismatic\r\n");
//			PxArticulationLink* link = tether->getStartLink();
//			sphericalJointToPrismatic( link->getInboundJoint() );
//
//		//	PxArticulationLink* oldLink		= tether->getStartLink();
//		//	PxVec3 oldLinkPos				= oldLink->getGlobalPose().p;
//		//	float oldLinkDist				= oldLinkPos.magnitude();
//		//	PxVec3 unitDirectionVec			= oldLinkPos.getNormalized();
//		//	PxVec3 oldLinkLinearVel			= oldLink->getLinearVelocity();
//		//	PxVec3 oldLinkAngularVel		= oldLink->getAngularVelocity();
//		//	PxVec3 oldLinkRadialVel			= oldLinkLinearVel.dot(unitDirectionVec)*unitDirectionVec;
//		//	PxVec3 oldLinkTangentialVel		= oldLinkLinearVel - oldLinkRadialVel;
//
//		//	PxVec3 newLinkPos			= oldLinkPos - unitDirectionVec*elementLength;
//		//	PxVec3 newLinkLinearVel			= oldLinkRadialVel + (driveTarget/oldLinkDist)*oldLinkTangentialVel;
//		//	PxVec3 newLinkAngularVel		= oldLinkAngularVel;
//
//		//	printf("Distance: %5.4f\n",(double)driveTarget);
//		//	printf("Old: px: %5.4f, py: %5.4f, pz: %5.4f\n",(double)oldLinkPos.x,(double)oldLinkPos.y,(double)oldLinkPos.z);
//		//	printf("Old: vx: %5.4f, vy: %5.4f, vz: %5.4f\n",(double)oldLinkLinearVel.x,(double)oldLinkLinearVel.y,(double)oldLinkLinearVel.z);
//		//	printf("Old: vr: %5.4f, vt: %5.4f\n",(double)oldLinkRadialVel.normalize(),(double)oldLinkTangentialVel.normalize());
//		//	printf("New: vx: %5.4f, vy: %5.4f, vz: %5.4f\n",(double)newLinkLinearVel.x,(double)newLinkLinearVel.y,(double)newLinkLinearVel.z);
//		//	printf("New: px: %5.4f, py: %5.4f, pz: %5.4f\n",(double)newLinkPos.x,(double)newLinkPos.y,(double)newLinkPos.z);
//
//		//	// Update mass of start link
//		//	// Start link mass is the weight of the whole tether spanning between the anchor and the first link and half the weight of the following tether element.
//		//	float startElementMass = (abs(driveTarget)+0.5f*elementLength)*characteristicMass;
//		//	PxRigidBodyExt::setMassAndUpdateInertia(*startLink, startElementMass);
//			//printf("Distance: %3.10f\r\n",(double)driveTarget);
		} //else{
//		printf("Drive target: %f\r\n",(double)driveTarget);
		if(anchorD6Joint != nullptr){
//			printf("Setting reeling velocity: %f\r\n",(double)reelingVelocity);
			bool autowake = true;
			PxVec3 zeroVec(PxZero);
			anchorD6Joint->setDrivePosition(PxTransform(PxVec3(0.0f,-driveTarget,0.0f)),autowake);
			//anchorD6Joint->setDriveVelocity(PxVec3(0.0f,-reelingVelocity,0.0f),zeroVec, autowake);
		}

//		bool isSleeping = gArticulation->isSleeping();
//		if(isSleeping){
		gArticulation->wakeUp();
//		}
		anchor->wakeUp();
		//}
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	gArticulation->release();
	gScene->release();
	gDispatcher->release();
	gPhysics->release();

	gCooking->release();
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
		float magnitude = 400.0f;
		//box->addForce(PxVec3(magnitude, magnitude, magnitude), PxForceMode::eIMPULSE,autowake);
		kite->addForce(PxVec3(magnitude, 0.0f, magnitude));
		// Apply force for tether force check
		//tether->getLink(1)->addForce(PxVec3(magnitude, 0.0f,0.0f), PxForceMode::eFORCE,autowake);
//		tether->getLink(1)->addForce(PxVec3(0.0f, -magnitude,0.0f), PxForceMode::eFORCE,autowake);
		//tether->getLink(0)->addForce(PxVec3(magnitude, 0.0f, 0.0f), PxForceMode::eFORCE,autowake);
		//tether->getLink(0)->addForce(PxVec3(magnitude, magnitude, magnitude), PxForceMode::eIMPULSE,autowake);
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
