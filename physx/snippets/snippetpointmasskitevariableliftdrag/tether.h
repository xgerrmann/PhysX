#ifndef TETHER_H
#define TETHER_H
//
// Author: X.G.Gerrmann
// Email: xander@xgerrmann.com
// ****************************************************************************
// This file contains all Tether related functionalities.
// ****************************************************************************

#include "common.h"

//using namespace physx;
namespace px = physx;


extern px::PxArticulationReducedCoordinate*	gArticulation;
extern bool flying;

const int nLinks						= 10;
const float elementLength				= 10.0f;
//const float characteristicMass			= 0.013f;	// % [kg/m]
const float characteristicMass			= 0.001f;	// % [kg/m]
const float radius						= 0.2f;


class Tether {
	public:
		px::PxArticulationLink* getStartLink();
		px::PxArticulationLink* getEndLink();
		void createRope();
		void addLink(px::PxVec3 pos);
		void removeLink();
		int getNbElements();
		Tether(int nLinks);
		void applyDrag();
		px::PxArticulationLink* getLink(int linkIndex);
	private:
		px::PxArticulationLink* endLink   = NULL;
		px::PxArticulationLink* startLink = NULL;

		int nbElements = 0;

		static const int maxNbElements = 64;

		px::PxArticulationLink* tetherElements[maxNbElements];
};

px::PxArticulationLink* Tether::getLink(int linkIndex){
	px::PxArticulationLink* links[1];
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

px::PxArticulationLink* Tether::getStartLink(){
	return startLink;
}

px::PxArticulationLink* Tether::getEndLink(){
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
	const px::PxU32 startIndex	= 0;
	px::PxArticulationLink* links[maxNbElements];
	gArticulation->getLinks(links, nbElements, startIndex);
	static float CD_tether = 0.5f;
	for(int ii = 0; ii<nbElements; ii++){
		px::PxVec3 linkVelocity = links[ii]->getLinearVelocity();
		px::PxVec3 apparentVelocity = - linkVelocity;
		if(flying){
			apparentVelocity = windVelocity - linkVelocity;
		}
		px::PxVec3 drag = apparentVelocity*CD_tether;
		links[ii]->addForce(-drag);
	}
}

void Tether::addLink(px::PxVec3 pos){
	// Number of elements may not exceed the set maximum
	if(nbElements >= maxNbElements){
		return;
	}
	// Update mass of current start element
	// Since it is now an intermediate link, its mass must equal the mass of one complete tether element
	if(startLink != nullptr){
		float startLinkMass = elementLength*characteristicMass;
		px::PxRigidBodyExt::setMassAndUpdateInertia(*startLink, startLinkMass);
	}

	px::PxArticulationLink* newLink = gArticulation->createLink(startLink, px::PxTransform(pos));

	px::PxShape* sphereShape = gPhysics->createShape(px::PxSphereGeometry(radius), *gMaterial);
	newLink->attachShape(*sphereShape);

	// Each link is initialized with half the mass of a tether element.
	float newLinkMass = 0.5f*characteristicMass*elementLength;	// % [kg]
	px::PxRigidBodyExt::setMassAndUpdateInertia(*newLink, newLinkMass);

	px::PxArticulationJointReducedCoordinate* joint = static_cast<px::PxArticulationJointReducedCoordinate*>(newLink->getInboundJoint());
	if(joint)	// Will be null for root link
	{
		// Joint must be located in child link. The child is the new link.
		joint->setParentPose(px::PxTransform(px::PxVec3(0.0f, -elementLength, 0.0f)));
		joint->setChildPose(px::PxTransform(px::PxVec3(0.0f)));

		// The joint type must be specified for reducedCoordinateArticulations
		joint->setJointType(px::PxArticulationJointType::eSPHERICAL);

		// Allowed motions must be specified
		joint->setMotion(px::PxArticulationAxis::eSWING1 , px::PxArticulationMotion::eFREE);
		joint->setMotion(px::PxArticulationAxis::eSWING2 , px::PxArticulationMotion::eFREE);
		joint->setMotion(px::PxArticulationAxis::eTWIST , px::PxArticulationMotion::eFREE);
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

	px::PxVec3 pos(0.0f,nLinks*elementLength,0.0f);

	// Create rope
	for(int i=0;i<nLinks;i++)
	{
		addLink(pos);
		pos.y -= elementLength;
	}
	std::cout << "nbElements: " << nbElements << std::endl;
}
#endif /* TETHER_H */
