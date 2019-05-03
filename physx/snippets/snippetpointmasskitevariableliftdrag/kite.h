#ifndef KITE_H
#define KITE_H
//
// Author: X.G.Gerrmann
// Email: xander@xgerrmann.com
// ****************************************************************************
// This file contains all kite related functionalities.
// ****************************************************************************


namespace px = physx;

px::PxPhysics*				gPhysics		= NULL;
px::PxCooking*				gCooking		= NULL;

px::PxDefaultCpuDispatcher*	gDispatcher		= NULL;
px::PxScene*				gScene			= NULL;

px::PxMaterial*				gMaterial		= NULL;

px::PxVec3 windVelocity(-20.0f,0.0f,0.0f);				// [m/s]
float rho = 1.225;		// [kg/m^3]

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
	px::PxVec3 lift = px::PxVec3(0.0f);	// [N]
	px::PxVec3 drag = px::PxVec3(0.0f);	// [N]

	public:
		Kite(px::PxArticulationLink* link);

		void getJointForce(px::PxVec3& jointForce, px::PxVec3& jointMoment);
		void addForce(px::PxVec3 force);

		void runAerodynamics();

		px::PxVec3 getPosition();
		px::PxQuat getOrientation();

		float getMass();
		px::PxVec3 getLift();
		px::PxVec3 getDrag();
		float getSurfaceArea();

		float cl_alpha(float alpha);
		float cd_alpha(float alpha);

	private:
		px::PxRigidDynamic*	kiteDynamic	= nullptr;
		px::PxFixedJoint*	kiteJoint	= nullptr;

		px::PxConvexMesh*	createMesh();

		void loadAeroData();
		float angleOfAttack(px::PxVec3 windVelocity);
};

Kite::Kite(px::PxArticulationLink* link){
	px::PxVec3 endLinkPos			= link->getGlobalPose().p;

	const float kiteDensity = kiteMass/(kiteWidth*kiteDepth*kiteHeight);
	px::PxVec3 kitePos	= endLinkPos;
	kitePos.y		+= 2*kiteHeight;

	px::PxConvexMesh* kiteMesh			= createMesh();
	px::PxShape* tmpShape	= gPhysics->createShape(px::PxBoxGeometry(px::PxVec3(0.5f)), *gMaterial);
	kiteDynamic			= px::PxCreateDynamic(*gPhysics, px::PxTransform(kitePos), *tmpShape, kiteDensity);
	px::PxShape* kiteShape	= px::PxRigidActorExt::createExclusiveShape(*kiteDynamic,px::PxConvexMeshGeometry(kiteMesh), *gMaterial);
	kiteDynamic->attachShape(*kiteShape);
	kiteDynamic->detachShape(*tmpShape);
	kiteJoint			= px::PxFixedJointCreate(*gPhysics, kiteDynamic, px::PxTransform(px::PxVec3(0.0f,-2*kiteHeight,0.0f)), link, px::PxTransform(px::PxVec3(0.0f)));
	gScene->addActor(*kiteDynamic);

	// Determine Cl and Cd from given ratio between CL and CD.
	// Assume: CR = sqrt( Cd^2+ Cl^2 ) = 1
	//CD_static				= 1.0f/px::PxSqrt(CL_CD*CL_CD + 1.0f);	// [-]
	//CL_static				= px::PxSqrt(1.0f - CD*CD);				// [-]
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
	return rad/px::PxPi*180;
}

float deg2Rad(float deg){
	return deg*px::PxPi/180;
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

px::PxVec3 Kite::getPosition(){
	return kiteDynamic->getGlobalPose().p;
}

px::PxQuat Kite::getOrientation(){
	return kiteDynamic->getGlobalPose().q;
}

px::PxConvexMesh* Kite::createMesh(){
	//static const px::PxVec3 convexVerts[] = {px::PxVec3(0,1,0),px::PxVec3(1,0,0),px::PxVec3(-1,0,0),px::PxVec3(0,0,1),px::PxVec3(0,0,-1)};
	static const px::PxVec3 convexVerts[] = {px::PxVec3(1,0,3),px::PxVec3(1,0,-3),px::PxVec3(-1,0,3),px::PxVec3(-1,0,-3)};

	px::PxConvexMeshDesc convexDesc;
	convexDesc.points.count     = 4;
	convexDesc.points.stride    = sizeof(px::PxVec3);
	convexDesc.points.data      = convexVerts;
	convexDesc.flags            = px::PxConvexFlag::eCOMPUTE_CONVEX;

	px::PxDefaultMemoryOutputStream buf;
	px::PxConvexMeshCookingResult::Enum result;
	if(!gCooking->cookConvexMesh(convexDesc, buf, &result)){
		return NULL;
	}
	px::PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
	px::PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);

	return convexMesh;
}

void Kite::getJointForce(px::PxVec3& jointForce, px::PxVec3& jointMoment){
	if(kiteJoint!=nullptr){
		kiteJoint->getConstraint()->getForce(jointForce, jointMoment);
	} else {
		jointForce = px::PxVec3(px::PxZero);
		jointMoment = px::PxVec3(px::PxZero);
	}
}

px::PxVec3 Kite::getLift(){
	return lift;
}

px::PxVec3 Kite::getDrag(){
	return drag;
}

float Kite::getSurfaceArea(){
	return kiteSurfaceArea;
}

void Kite::addForce(px::PxVec3 force){
	static const bool autowake = true;
	kiteDynamic->addForce(force, px::PxForceMode::eFORCE,autowake);
}

float Kite::angleOfAttack(px::PxVec3 windVelocity){
	px::PxVec3 kiteSpeed= kiteDynamic->getLinearVelocity();
	px::PxVec3 va		= windVelocity - kiteSpeed;

	// Determine angle of attack
	px::PxQuat kiteOrientation = getOrientation();
	px::PxVec3 ex(1,0,0);
	px::PxVec3 ey(0,1,0);
	px::PxVec3 ez(0,0,1);
	px::PxVec3 ex_local = kiteOrientation.rotate(ex);
	px::PxVec3 ez_local = kiteOrientation.rotate(ez);
	printf("eX local: X: %f, Y: %f, Z; %f\r\n",(double)ex_local.x,(double)ex_local.y,(double)ex_local.z);

	px::PxVec3 va_proj  = -(va - va.dot(ez_local)*ez); // Projected Va in opposite direction
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
	px::PxVec3 kiteSpeed= kiteDynamic->getLinearVelocity();
//	printf("KiteSpeed: %5.5f, %5.5f, %5.5f\r\n",(double) kiteSpeed.x, (double)kiteSpeed.y, (double)kiteSpeed.z);

	// Apparent windspeed (projected in the local x, y plane)
	px::PxVec3 va		= windVelocity - kiteSpeed;
	px::PxQuat kiteOrientation = getOrientation();
	px::PxVec3 ez(0,0,1);
	px::PxVec3 ez_local = kiteOrientation.rotate(ez);
	px::PxVec3 va_proj  = va-va.dot(ez_local)*ez;

	float Cl, Cd;
	if(aero_mode == MODE_FIXED){
		Cl = static_CL;
		Cd = static_CD;
	} else{
		float alphaRad = angleOfAttack(windVelocity);
		printf("AoA: %f\n\r",(double)rad2Deg(alphaRad));
		Cl = cl_alpha(alphaRad);
		Cd = cd_alpha(alphaRad);
	}

	float va2		= va_proj.magnitudeSquared();

	px::PxVec3 liftDirection = va_proj.cross(ez).getNormalized();
	px::PxVec3 dragDirection = va.getNormalized();

	lift = half*rho*va2*kiteSurfaceArea*Cl*liftDirection;
	drag = half*rho*va2*kiteSurfaceArea*Cd*dragDirection;

	addForce(lift);
	addForce(drag);
	printf("LIFT X: %5.5f, Y: %5.5f, Z: %5.5f\r\n",(double)lift.x,(double)lift.y,(double)lift.z);
	printf("DRAG X: %5.5f, Y: %5.5f, Z: %5.5f\r\n",(double)drag.x,(double)drag.y,(double)drag.z);
	printf("Cl: %5.5f, Cd: %5.5f\r\n",(double)Cl,(double)Cd);
}
#endif /* KITE_H */
