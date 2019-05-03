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

#include <cassert>

using namespace physx;

// Forward declarations
class Kite;
class Tether;
Kite* kite;
Tether* tether;

void cleanupPhysics(bool);

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
//const float characteristicMass			= 0.013f;	// % [kg/m]
const float characteristicMass			= 0.001f;	// % [kg/m]
const float radius						= 0.2f;

float maxReelingVelocity				= 100;				// [m/s]
float dReelingVelocity					= 10.0f/slowDown/subStepCount;	// [m/s]
float reelingVelocity					= 0.0f;				// [m/s]

enum reelingDirection {reelIn = 0, reelOut = 1, None = 2};

enum reelingDirection currentReelingDirection = None;

PxRigidDynamic* anchor					= nullptr;

PxFixedJoint* anchorFixedJoint			= nullptr;
PxD6Joint* anchorD6Joint				= nullptr;

float CL_CD								= 0.0f;



PxArticulationJointReducedCoordinate* driveJoint			= nullptr;

bool flying								= true;		// [-]
//float windspeed							= 40;		// [m/s]
float liftForce							= 5000;		// [N]
PxVec3 windVelocity(-20.0f,0.0f,0.0f);				// [m/s]
float rho = 1.225;		// [kg/m^3]


char fpath[264];
std::fstream file;

enum aerodynamic_mode { MODE_FIXED, MODE_VARIABLE };

class Kite {
	const float kiteSurfaceArea	= 10;	// [m^2]
	const float kiteMass		= 50.0f;	// [kg]
	const float kiteWidth		= 2.0f;		// [m]
	const float kiteHeight		= 0.5f;		// [m]
	const float kiteDepth		= 1.0f;		// [m]

	float static_CD				= 1.0f;		// [-]
	float static_CL				= 1.0f;		// [-]

	const static int n_deg_max	= 360;		// [-]
	int alpha[n_deg_max];
	float Cl_variable[n_deg_max], Cd_variable[n_deg_max];
	
	aerodynamic_mode aero_mode	= MODE_VARIABLE;
	//aerodynamic_mode aero_mode	= MODE_FIXED;

	// Store most recently applied lift and drag force
	PxVec3 lift = PxVec3(0.0f);	// [N]
	PxVec3 drag = PxVec3(0.0f);	// [N]

	public:
		Kite(PxArticulationLink* link);

		void getJointForce(PxVec3& jointForce, PxVec3& jointMoment);
		void addForce(PxVec3 force);

		void runAerodynamics();

		PxVec3 getPosition();
		PxQuat getOrientation();

		float getMass();
		PxVec3 getLift();
		PxVec3 getDrag();
		float getSurfaceArea();

		float cl_alpha(float alpha);
		float cd_alpha(float alpha);

	private:
		PxRigidDynamic*	kiteDynamic	= nullptr;
		PxFixedJoint*	kiteJoint	= nullptr;

		PxConvexMesh*	createMesh();

		void loadAeroData();
		float angleOfAttack(PxVec3 windVelocity);
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

	// Determine Cl and Cd from given ratio between CL and CD.
	// Assume: CR = sqrt( Cd^2+ Cl^2 ) = 1
	//CD_static				= 1.0f/PxSqrt(CL_CD*CL_CD + 1.0f);	// [-]
	//CL_static				= PxSqrt(1.0f - CD*CD);				// [-]
	//printf("CL: %5.5f, CD: %5.5f\r\n",(double)CL,(double)CD);

	loadAeroData();
}

void Kite::loadAeroData(){
	// Assumes csv data is in the format
	// Angle of attack (int) , Cl (float), Cd(float)
	const char fname[] = "/home/xander/Google Drive/Thesis/src/PhysX/physx/snippets/snippetpointmasskitevariableliftdrag/aeroData.csv";
	//const char fname[] = "/home/xander/Google Drive/Thesis/src/PhysX/physx/snippets/snippetpointmasskitevariableliftdrag/aeroData_fixed.csv";
	const char delim = ',';
	std::fstream  aeroDataFile;
	printf("Fname: %s\r\n",fname);
	aeroDataFile.open(fname,std::ios::in);
	std::string line, value;
	if(aeroDataFile.is_open()){
		for(int lineCounter=0; std::getline(aeroDataFile, line); lineCounter++)
		{
			printf("%s\r\n",line.c_str());
			if(lineCounter>=n_deg_max){
				// Arrays can contain max n_deg_max values
				throw std::runtime_error("File contains more than allowed number of lines (360).");
			}
			std::istringstream iss(line);
			for(int valueCounter=0; std::getline(iss, value, delim); valueCounter++){
				printf("%s\r\n",value.c_str());
				switch(valueCounter){
					case 0: alpha[lineCounter] = std::stoi(value);			break;
					case 1: Cl_variable[lineCounter] = std::stof(value);	break;
					case 2: Cd_variable[lineCounter] =  std::stof(value);	break;
					default: throw std::runtime_error("Something is wrong with the *.csv file, more than 3 columns detected.");
				}
			}
		}
		aeroDataFile.close();
	} else {
		throw std::runtime_error("Could not open file.");
	}
}

float rad2Deg(float rad){
	return rad/PxPi*180;
}

float deg2Rad(float deg){
	return deg*PxPi/180;
}

// Wrap angle [-180, 180)
// https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
float constrainAngleDeg(float x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

float Kite::cl_alpha(float alphaRad){
	float CL = 0.0f;
	if(aero_mode == MODE_FIXED){
		CL = static_CL;
	} else{ // aero_mode == MODE_VARIABLE
		float alphaDeg = constrainAngleDeg(rad2Deg(alphaRad));
		int alphaDegInt = (int) round(alphaDeg);
		int angleIndex  = alphaDegInt + 180;
		CL = Cl_variable[angleIndex];
	}
	return CL;
}

float Kite::cd_alpha(float alphaRad){
	float CD = 0.0f;
	if(aero_mode == MODE_FIXED){
		CD = static_CD;
	} else{ // aero_mode == MODE_VARIABLE
		float alphaDeg = constrainAngleDeg(rad2Deg(alphaRad));
		int alphaDegInt = (int) round(alphaDeg);
		int angleIndex  = alphaDegInt + 180;
		CD = Cd_variable[angleIndex];
	}
	return CD;
}

float Kite::getMass(){
	return kiteMass;
}

PxVec3 Kite::getPosition(){
	return kiteDynamic->getGlobalPose().p;
}

PxQuat Kite::getOrientation(){
	return kiteDynamic->getGlobalPose().q;
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

PxVec3 Kite::getLift(){
	return lift;
}

PxVec3 Kite::getDrag(){
	return drag;
}

float Kite::getSurfaceArea(){
	return kiteSurfaceArea;
}

void Kite::addForce(PxVec3 force){
	static const bool autowake = true;
	kiteDynamic->addForce(force, PxForceMode::eFORCE,autowake);
}

float Kite::angleOfAttack(PxVec3 windVelocity){
	PxVec3 kiteSpeed= kiteDynamic->getLinearVelocity();
	PxVec3 va		= windVelocity - kiteSpeed;

	// Determine angle of attack
	PxQuat kiteOrientation = getOrientation();
	PxVec3 ex(1,0,0);
	PxVec3 ey(0,1,0);
	PxVec3 ez(0,0,1);
	PxVec3 ex_local = kiteOrientation.rotate(ex);
	PxVec3 ez_local = kiteOrientation.rotate(ez);
	printf("eX local: X: %f, Y: %f, Z; %f\r\n",(double)ex_local.x,(double)ex_local.y,(double)ex_local.z);

	PxVec3 va_proj  = -(va - va.dot(ez_local)*ez); // Projected Va in opposite direction
	printf("va_proj : X: %f, Y: %f, Z; %f\r\n",(double)va_proj.x,(double)va_proj.y,(double)va_proj.z);
	float numerator = va_proj.dot(ex_local);
	float denominator = va_proj.magnitude()*ex_local.magnitude();
	printf("num : %f, den: %f\r\n",(double)numerator,(double)denominator);
	float alphaRad  = acos(numerator/denominator);
	printf("alphaRad : %f\r\n",(double)alphaRad);

	return alphaRad;
}

void Kite::runAerodynamics(){
//	printf("WindSpeed: %5.5f, %5.5f, %5.5f\r\n",(double) windVelocity.x, (double)windVelocity.y, (double)windVelocity.z);
	float half = 0.5;		// [-]
	PxVec3 kiteSpeed= kiteDynamic->getLinearVelocity();
//	printf("KiteSpeed: %5.5f, %5.5f, %5.5f\r\n",(double) kiteSpeed.x, (double)kiteSpeed.y, (double)kiteSpeed.z);

	// Apparent windspeed (projected in the local x, y plane)
	PxVec3 va		= windVelocity - kiteSpeed;
	PxQuat kiteOrientation = getOrientation();
	PxVec3 ez(0,0,1);
	PxVec3 ez_local = kiteOrientation.rotate(ez);
	PxVec3 va_proj  = va-va.dot(ez_local)*ez;

	float Cl, Cd;
	if(aero_mode == MODE_FIXED){
		Cl = static_CL;
		Cd = static_CD;
	} else{
		float alphaRad = kite->angleOfAttack(windVelocity);
		printf("AoA: %f\n\r",(double)rad2Deg(alphaRad));
		Cl = cl_alpha(alphaRad);
		Cd = cd_alpha(alphaRad);
	}

	float va2		= va_proj.magnitudeSquared();

	PxVec3 liftDirection = va_proj.cross(ez).getNormalized();
	PxVec3 dragDirection = va.getNormalized();

	lift = half*rho*va2*kiteSurfaceArea*Cl*liftDirection;
	drag = half*rho*va2*kiteSurfaceArea*Cd*dragDirection;

	addForce(lift);
	addForce(drag);
	printf("LIFT X: %5.5f, Y: %5.5f, Z: %5.5f\r\n",(double)lift.x,(double)lift.y,(double)lift.z);
	printf("DRAG X: %5.5f, Y: %5.5f, Z: %5.5f\r\n",(double)drag.x,(double)drag.y,(double)drag.z);
	printf("Cl: %5.5f, Cd: %5.5f\r\n",(double)Cl,(double)Cd);
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
	static float CD_tether = 0.5f;
	for(int ii = 0; ii<nbElements; ii++){
		PxVec3 linkVelocity = links[ii]->getLinearVelocity();
		PxVec3 apparentVelocity = - linkVelocity;
		if(flying){
			apparentVelocity = windVelocity - linkVelocity;
		}
		PxVec3 drag = apparentVelocity*CD_tether;
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
	//while(added){}
	for(PxU32 i=0; i < subStepCount; i++){
		if(flying){
//			kite->addForce(PxVec3(0.0f,liftForce, 0.0f));
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
