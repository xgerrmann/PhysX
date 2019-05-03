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
#include <sstream>

#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

#include "common.h"
#include "kite.h"
#include "tether.h"

#include <cassert>

using namespace physx;

Tether* tether;
Kite* kite;

void cleanupPhysics(bool);

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation		= NULL;

PxPvd*                  gPvd			= NULL;

PxArticulationCache* articulationCache	= NULL;

PxU32 subStepCount						= 20.0f;
const float slowDown					= 1.0f;
const float dt							= 1.0f/60.0f/slowDown/subStepCount;
const float gravity						= 9.81f;

float maxReelingVelocity				= 100;				// [m/s]
float dReelingVelocity					= 10.0f/slowDown/subStepCount;	// [m/s]
float reelingVelocity					= 0.0f;				// [m/s]

enum reelingDirection {reelIn = 0, reelOut = 1, None = 2};

enum reelingDirection currentReelingDirection = None;

float CL_CD								= 0.0f;

char fpath[264];
std::fstream file;


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
	// Setup filename
	float ms = std::chrono::system_clock::now().time_since_epoch().count();
	sprintf(fpath,"/home/xander/Google Drive/Thesis/src/2D_lift_drag_analysis/data/data_CL_CD_%.3f_%f",(double)CL_CD, (double)ms);
	// Save data
	std::cout << fpath << std::endl;
	std::ofstream file;
	file.open(fpath, std::ios_base::out);
	const static int length_head = 12;
	file << "Length_head: "			<<  length_head << std::endl;
	file << "Timestep: "			<<  dt << std::endl;
	file << "CharacteristicMass: "	<<  characteristicMass << std::endl;
	file << "ElementLength: "		<<  elementLength << std::endl;
	file << "kiteMass: "			<<  kite->getMass() << std::endl;
	file << "gravity: "				<<  gravity << std::endl;
	file << "CL/CD: "				<<  CL_CD << std::endl;
	file << "kiteArea: "			<<  kite->getSurfaceArea() << std::endl;
	file << "airDensity: "			<<  rho<< std::endl;
	file << "windVelocity_x: "		<<  windVelocity.x << std::endl;
	file << "windVelocity_y: "		<<  windVelocity.y << std::endl;
	file << "windVelocity_z: "		<<  windVelocity.z << std::endl;

	// CSV header
	file<< "time,"
		<< "nElements,"
		<< "liftForce_x,"
		<< "liftForce_y,"
		<< "liftForce_z,"
		<< "dragForce_x,"
		<< "dragForce_y,"
		<< "dragForce_z,"
		<< "kitePosition_x,"
		<< "kitePosition_y,"
		<< "kitePosition_z"
		<< std::endl;
	file.close();
}

void logForce(float t){

	PxVec3 kitePosition, liftForce, dragForce;
	kitePosition = kite->getPosition();
	liftForce = kite->getLift();
	dragForce = kite->getDrag();

	file.open(fpath, std::ios_base::app);
	// Save data
	file<< t << ","
		<< nLinks << ","
		<< liftForce.x << ","
		<< liftForce.y << ","
		<< liftForce.z << ","
		<< dragForce.x << ","
		<< dragForce.y << ","
		<< dragForce.z << ","
		<< kitePosition.x << ","
		<< kitePosition.y << ","
		<< kitePosition.z
		<< std::endl;
	file.close();
	file<< "time,"
		<< "nElements,"
		<< "liftForce_x,"
		<< "liftForce_y,"
		<< "liftForce_z,"
		<< "dragForce_x,"
		<< "dragForce_y,"
		<< "dragForce_z,"
		<< "kitePosition_x,"
		<< "kitePosition_y,"
		<< "kitePosition_z"
		<< std::endl;
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

	initFile();
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
void printElevationAngle(){
	PxVec3 kitePosition = kite->getPosition();
	printf("KitePosition: X: %5.5f, y: %5.5f, z: %5.5f\r\n", (double)kitePosition.x, (double)kitePosition.y,(double)kitePosition.z);
	// In 2D case, the position of the kite in Z is zero and the elevation angle is determined by the x and y coordinate.
	float fraction = kitePosition.y / (-kitePosition.x);
	float elevationAngle = PxAtan(fraction);
	printf("ElevationAngle: :%5.5f\r\n",(double)elevationAngle/3.1416*180);
}
void stepPhysics(bool /*interactive*/)
{
//	const static float maxtime = 20.0f; // [s]
	const static std::chrono::milliseconds maxtime = std::chrono::milliseconds(40000); // [ms]
	static std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> t_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	//static bool added = false;
	static float t_elapsed_f = 0.0f;
	//printf("Distance: %5.6f\r\n",(double)driveTarget);
	std::chrono::milliseconds t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-t_start);
	if(t_elapsed > maxtime){
		exit(0);
	}
	static float driveTarget = elementLength;//tether->getStartLink()->getGlobalPose().p.normalize();
//	std::cout << "Chrono: " << t_elapsed_ch.count() << ", simulation: " << t_elapsed_f << "dt: " << dt << std::endl;
	for(PxU32 i=0; i < subStepCount; i++){
		if(flying){
			kite->runAerodynamics();
		}
		logForce(t_elapsed_f);
		//printPositions();
		printElevationAngle();
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
			// Change joint pose
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

		}
		if(anchorD6Joint != nullptr){
			bool autowake = true;
			PxVec3 zeroVec(PxZero);
			anchorD6Joint->setDrivePosition(PxTransform(PxVec3(0.0f,-driveTarget,0.0f)),autowake);
		}

		bool isSleeping = gArticulation->isSleeping();
		if(isSleeping){
			gArticulation->wakeUp();
		}
		anchor->wakeUp();
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
		kite->addForce(PxVec3(magnitude, 0.0f, magnitude));
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
