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

const int n_links						= 40;
const float scale						= 0.25f;
const float radius						= 0.5f*scale;
const float halfHeight					= 1.0f*scale;
const PxU32 nbCapsules					= (PxU32) n_links;
const float capsuleMass					= 1.0f;

const float gravity						= 9.81f;

float get_positions(float* posx, float* posy){
	// Get positions
	const PxU32 startIndex	= 0;
	float error				= 0.0f;
	PxArticulationLink* links[n_links];
	gArticulation->getLinks(links, nbCapsules, startIndex);
	for(PxU32 ii = 0; ii< n_links; ii++){
		PxVec3 pos = links[ii]->getGlobalPose().p;
		error += sqrt(pow(posx[ii]-pos.x,2) + pow(posy[ii]-pos.y,2));
		posx[ii] = pos.x;
		posy[ii] = pos.y;
	}
	std::cout.precision(10);
	std::cout << "Error: " << error << std::endl;
	return error;
}

void save_positions(float* posx, float* posy, float force){
	// Save data
	const static char* dir = "/home/xander/Google Drive/Thesis/src/catenary_analysis/data/";
	char fname[40];
	sprintf(fname,"%sforce_%d",dir,(int)force);
	std::cout << fname << std::endl;
	std::ofstream file;
	file.open(fname);
	file << "Number: "		<<  n_links << std::endl;
	file << "Mass: "		<<  capsuleMass << std::endl;
	file << "HalfHeight: "	<<  halfHeight << std::endl;
	file << "Scale: "		<<  scale << std::endl;
	file << "Force: "		<<  force << std::endl;
	file << "Gravity: "		<<  gravity << std::endl;
	// CSV header
	file << "x_coor" 		<< "," << "y_coor" << std::endl;
	for(int ii = 0; ii<(int)n_links; ii++){
		file << posx[ii] << "," << posy[ii] << std::endl;
	}
	file.close();
}


void applyDrag(){
	const PxU32 n_links		= 40;
	const PxU32 startIndex	= 0;
	PxArticulationLink* links[n_links];
	gArticulation->getLinks(links, n_links, startIndex);
	static float CD = 2.0f;
	for(PxU32 ii = 0; ii<n_links; ii++){
		PxVec3 vel = links[ii]->getLinearVelocity();
		PxVec3 drag = vel*CD;
		links[ii]->addForce(-drag);
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

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	gArticulation = gPhysics->createArticulation();

	// Stabilization can create artefacts on jointed objects so we just disable it
	gArticulation->setStabilizationThreshold(0.0f);

	gArticulation->setMaxProjectionIterations(16);
	gArticulation->setSeparationTolerance(0.001f);


	const PxVec3 initPos(0.0f, 24.0f, 0.0f);
	PxVec3 pos = initPos;
	PxShape* capsuleShape = gPhysics->createShape(PxCapsuleGeometry(radius, halfHeight), *gMaterial);
	PxArticulationLink* firstLink = NULL;
	PxArticulationLink* parent = NULL;

	const bool overlappingLinks = true;	// Change this for another kind of rope

	// Create rope
	for(PxU32 i=0;i<nbCapsules;i++)
	{
		PxArticulationLink* link = gArticulation->createLink(parent, PxTransform(pos));
		if(!firstLink)
			firstLink = link;

		link->attachShape(*capsuleShape);
		PxRigidBodyExt::setMassAndUpdateInertia(*link, capsuleMass);

		PxArticulationJointBase* joint = link->getInboundJoint();
		if(joint)	// Will be null for root link
		{
			if(overlappingLinks)
			{
				joint->setParentPose(PxTransform(PxVec3(halfHeight, 0.0f, 0.0f)));
				joint->setChildPose(PxTransform(PxVec3(-halfHeight, 0.0f, 0.0f)));
			}
			else
			{
				joint->setParentPose(PxTransform(PxVec3(radius + halfHeight, 0.0f, 0.0f)));
				joint->setChildPose(PxTransform(PxVec3(-radius - halfHeight, 0.0f, 0.0f)));
			}
		}

		if(overlappingLinks)
			pos.x += (radius + halfHeight*2.0f);
		else
			pos.x += (radius + halfHeight) * 2.0f;
		parent = link;
	}
	lastLink = parent;

	gScene->addArticulation(*gArticulation);

	// Attach articulation to static world
	{
		PxShape* anchorShape = gPhysics->createShape(PxSphereGeometry(0.05f), *gMaterial);
		PxRigidStatic* anchor = PxCreateStatic(*gPhysics, PxTransform(initPos), *anchorShape);
		gScene->addActor(*anchor);
		PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, anchor, PxTransform(PxVec3(0.0f)), firstLink, PxTransform(PxVec3(0.0f)));
		PX_UNUSED(j);
	}
}

void stepPhysics(bool /*interactive*/)
{
	static bool query_force	= true;
	static bool simulate	= false;
	static float force		= 0.0f;
	
	static float posx[n_links];
	static float posy[n_links];
	float error				= 1000.0f;
	float error_thresh		= 1e-4f;

	if(query_force){
		//// Report
		std::cout << "Give force [N] and press enter:\n" << std::endl;
		std::cin >> force;
		query_force	= false;
		simulate	= true;
	}

	if(simulate){
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);

		// Apply drag to tether elements
		applyDrag();

		// Apply Horizontal force to tether
		lastLink->addForce(PxVec3(force, 0.0f, 0.0f));
	}

	error = get_positions(posx, posy);

	if(error < error_thresh){
		save_positions(posx, posy, force);
		simulate	= false;
		query_force = true;
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
