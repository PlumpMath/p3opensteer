// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2005, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// OpenSteer Boids
// 
// 09-26-02 cwr: created 
//
//
// ----------------------------------------------------------------------------
/**
 * \file /Ely/include/Support/OpenSteerLocal/PlugIn_Boids.h
 *
 * \date 2014-01-23 
 * \author consultit
 */

#ifndef PLUGIN_BOIDS_H_
#define PLUGIN_BOIDS_H_

#include <sstream>
#include <OpenSteer/Proximity.h>
#include <OpenSteer/Color.h>
#include <OpenSteer/UnusedParameter.h>
#include <OpenSteer/PlugIn.h>
#include "common.h"

///#ifndef NO_LQ_BIN_STATS
///#include <iomanip> // for setprecision
///#include <limits> // for numeric_limits::max()
///#endif // NO_LQ_BIN_STATS

namespace ossup
{

// Include names declared in the OpenSteer namespace into the
// namespaces to search to find names.
using namespace OpenSteer;

// ----------------------------------------------------------------------------

typedef OpenSteer::AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef OpenSteer::AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;

// ----------------------------------------------------------------------------

/**
 * \note: Public class members for tweaking:
 * - \fn void setFlockParameters(
 * 				float separationRadius, float separationAngle, float separationWeight,
 * 				float alignmentRadius,float alignmentAngle, float alignmentWeight,
 * 				float cohesionRadius, float cohesionAngle, float cohesionWeight)
 */
template<typename Entity>
class Boid: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a boid: an STL vector of Boid pointers
	typedef typename std::vector<Boid<Entity>*> groupType;

	// constructor
///	Boid(ProximityDatabase& pd)
	Boid()
	{
		// allocate a token for this boid in the proximity database
		proximityToken = NULL;
///		newPD(pd);

// reset all boid state
		reset();

		setFlockParameters(5.0f, -0.707f, 12.0f, 7.5f, 0.7f, 8.0f, 9.0f, -0.15f,
				8.0f);
	}

	// destructor
	~Boid()
	{
		// delete this boid's token in the proximity database
		delete proximityToken;
	}

	// reset state
	void reset(void)
	{
		// reset the vehicle
		SimpleVehicle::reset();
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();

		// steering force is clipped to this magnitude
///		setMaxForce(27);

		// velocity is clipped to this magnitude
///		setMaxSpeed(9);

		// initial slow speed
///		setSpeed(maxSpeed() * 0.3f);

///		// randomize initial orientation
///		this->regenerateOrthonormalBasisUF(RandomUnitVector());

///		// randomize initial position
///		this->setPosition(RandomVectorInUnitRadiusSphere() * 20);

///		// notify proximity database that our position has changed
///		proximityToken->updateForNewPosition(position());
	}

#ifdef OS_DEBUG
	// draw this boid into the scene
	void draw(void)
	{
		drawBasic3dSphericalVehicle(*this, gGray70);
		// this->drawTrail ();
	}
#endif

	// per frame simulation update
	void update(const float currentTime, const float elapsedTime)
	{
		OPENSTEER_UNUSED_PARAMETER(currentTime);

		// steer to flock and avoid obstacles if any
		this->applySteeringForce(steerToFlock(), elapsedTime);

		// wrap around to contrain boid within the spherical boundary
		sphericalWrapAround();

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

		// notify proximity database that our position has changed
		proximityToken->updateForNewPosition(this->position());
	}

	// basic flocking
	Vec3 steerToFlock(void)
	{
		// avoid obstacles if needed
		// XXX this should probably be moved elsewhere
		const Vec3 avoidance = this->steerToAvoidObstacles(1.0f, *obstacles);
		if (avoidance != Vec3::zero)
			return avoidance;

///		const float separationRadius = 5.0f;
///		const float separationAngle = -0.707f;
///		const float separationWeight = 12.0f;
///
///		const float alignmentRadius = 7.5f;
///		const float alignmentAngle = 0.7f;
///		const float alignmentWeight = 8.0f;
///
///		const float cohesionRadius = 9.0f;
///		const float cohesionAngle = -0.15f;
///		const float cohesionWeight = 8.0f;
///
///		const float maxRadius = maxXXX(separationRadius,
///				maxXXX(alignmentRadius, cohesionRadius));

		// find all flockmates within maxRadius using proximity database
		neighbors->clear();
		proximityToken->findNeighbors(this->position(), maxRadius, *neighbors);

///#ifndef NO_LQ_BIN_STATS
///		// maintain stats on max/min/ave neighbors per boids
///		size_t count = neighbors->size();
///		if (maxNeighbors < count)
///			maxNeighbors = count;
///		if (minNeighbors > count)
///			minNeighbors = count;
///		totalNeighbors += count;
///#endif // NO_LQ_BIN_STATS

		// determine each of the three component behaviors of flocking
		const Vec3 separation = this->steerForSeparation(separationRadius,
				separationAngle, *neighbors);
		const Vec3 alignment = this->steerForAlignment(alignmentRadius,
				alignmentAngle, *neighbors);
		const Vec3 cohesion = this->steerForCohesion(cohesionRadius, cohesionAngle,
				*neighbors);

		// apply weights to components (save in variables for annotation)
		const Vec3 separationW = separation * separationWeight;
		const Vec3 alignmentW = alignment * alignmentWeight;
		const Vec3 cohesionW = cohesion * cohesionWeight;

#ifdef OS_DEBUG
		// annotation
		// const float s = 0.1;
		// this->annotationLine (this->position(), this->position() + (separationW * s), gRed);
		// this->annotationLine (this->position(), this->position() + (alignmentW  * s), gOrange);
		// this->annotationLine (this->position(), this->position() + (cohesionW   * s), gYellow);
#endif

		return separationW + alignmentW + cohesionW;
	}

	// constrain this boid to stay within spherical boundary.
	void sphericalWrapAround(void)
	{
		// when outside the sphere
		if ((this->position() - worldCenter).length() > worldRadius)
		{
			// wrap around (teleport)
			this->setPosition(
					this->position().sphericalWrapAround(worldCenter, worldRadius));
		}
	}

	// ---------------------------------------------- xxxcwr111704_terrain_following
	// control orientation for this boid
	void regenerateLocalSpace(const Vec3& newVelocity, const float elapsedTime)
	{
		// 3d flight with banking
		this->regenerateLocalSpaceForBanking(newVelocity, elapsedTime);

		// // follow terrain surface
		// regenerateLocalSpaceForTerrainFollowing (newVelocity, elapsedTime);
	}

	// XXX experiment:
	// XXX   herd with terrain following
	// XXX   special case terrain: a sphere at the origin, radius 40
	void regenerateLocalSpaceForTerrainFollowing(const Vec3& newVelocity,
			const float /* elapsedTime */)
	{

		// XXX this is special case code, these should be derived from arguments //
		const Vec3 surfaceNormal = this->position().normalize();                    //
		const Vec3 surfacePoint = surfaceNormal * 40.0f;                      //
		// XXX this is special case code, these should be derived from arguments //

		const Vec3 newUp = surfaceNormal;
		const Vec3 newPos = surfacePoint;
		const Vec3 newVel = newVelocity.perpendicularComponent(newUp);
		const float newSpeed = newVel.length();
		const Vec3 newFor = newVel / newSpeed;

		this->setSpeed(newSpeed);
		this->setPosition(newPos);
		this->setUp(newUp);
		this->setForward(newFor);
		this->setUnitSideFromForwardAndUp();
	}
	// ---------------------------------------------- xxxcwr111704_terrain_following

	// switch to new proximity database -- just for demo purposes
	void newPD(ProximityDatabase& pd)
	{
		// delete this boid's token in the old proximity database
		delete proximityToken;

		// allocate a token for this boid in the proximity database
		proximityToken = pd.allocateToken(this);
	}

	// group of all obstacles to be avoided by each Boid
///	static ObstacleGroup obstacles;
	ObstacleGroup* obstacles;

	// a pointer to this boid's interface object for the proximity database
	ProximityToken* proximityToken;

	// allocate one and share amoung instances just to save memory usage
	// (change to per-instance allocation to be more MP-safe)
///	static AVGroup neighbors;
	AVGroup* neighbors;

///	static float worldRadius;
	float worldRadius;
	Vec3 worldCenter;

	// xxx perhaps this should be a call to a general purpose annotation for
	// xxx "local xxx axis aligned box in XZ plane" -- same code in in
	// xxx CaptureTheFlag.cpp
	void annotateAvoidObstacle(const float minDistanceToCollision)
	{
#ifdef OS_DEBUG
		const Vec3 boxSide = this->side() * this->radius();
		const Vec3 boxFront = this->forward() * minDistanceToCollision;
		const Vec3 FR = this->position() + boxFront - boxSide;
		const Vec3 FL = this->position() + boxFront + boxSide;
		const Vec3 BR = this->position() - boxSide;
		const Vec3 BL = this->position() + boxSide;
		const Color white(1, 1, 1);
		this->annotationLine(FR, FL, white);
		this->annotationLine(FL, BL, white);
		this->annotationLine(BL, BR, white);
		this->annotationLine(BR, FR, white);
#endif

		///call parent::annotateAvoidObstacle
		VehicleAddOnMixin<SimpleVehicle, Entity>::annotateAvoidObstacle(
				minDistanceToCollision);
	}

	void setFlockParameters(float sR, float sA, float sW,
			float aR, float aA,	float aW,
			float cR, float cA,	float cW)
	{
		separationRadius = sR;
		separationAngle = sA;
		separationWeight = sW;
		alignmentRadius = aR;
		alignmentAngle = aA;
		alignmentWeight = aW;
		cohesionRadius = cR;
		cohesionAngle = cA;
		cohesionWeight = cW;
		maxRadius = maxXXX(separationRadius,
				maxXXX(alignmentRadius, cohesionRadius));
	}

///#ifndef NO_LQ_BIN_STATS
///	static size_t minNeighbors, maxNeighbors, totalNeighbors;
///#endif // NO_LQ_BIN_STATS

private:
	float separationRadius;
	float separationAngle;
	float separationWeight;
	float alignmentRadius;
	float alignmentAngle;
	float alignmentWeight;
	float cohesionRadius;
	float cohesionAngle;
	float cohesionWeight;
	float maxRadius;

};

//Boid externally updated.
template<typename Entity>
class ExternalBoid: public Boid<Entity>
{
public:

	void update(const float currentTime, const float elapsedTime)
	{
		OPENSTEER_UNUSED_PARAMETER(currentTime);

		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

		// notify proximity database that our position has changed
		this->proximityToken->updateForNewPosition(this->position());
	}
};

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo

/**
 * \note: Public class members for tweaking:
 * - \var worldCenter, worldRadius: specify the "world sphere" boundary.
 * - \fn void nextPD(): cycles through various types of proximity databases
 * (default: LQProximityDatabase).
 */
template<typename Entity>
class BoidsPlugIn: public PlugIn
{
public:

	//set default world radius in constructor
	BoidsPlugIn(): worldRadius(10.0f), worldCenter(Vec3::zero)
	{
	}

	const char* name(void)
	{
		return "Boids";
	}

	float selectionOrderSortKey(void)
	{
		return 0.03f;
	}

	virtual ~BoidsPlugIn()
	{
	} // be more "nice" to avoid a compiler warning

	void open(void)
	{
		// make the database used to accelerate proximity queries
		cyclePD = -1;
		pd = NULL;
		nextPD();

///		// make default-sized flock
///		population = 0;
///		for (int i = 0; i < gBoidsNum; i++)
///			addBoidToFlock();

///		// set up obstacles
///		initObstacles();
	}

	void update(const float currentTime, const float elapsedTime)
	{
///#ifndef NO_LQ_BIN_STATS
///		Boid::maxNeighbors = Boid::totalNeighbors = 0;
///		Boid::minNeighbors = std::numeric_limits<int>::max();
///#endif // NO_LQ_BIN_STATS

		// update flock simulation for each boid
		iterator iter;
		for (iter = flock.begin(); iter != flock.end(); ++iter)
		{
			(*iter)->update(currentTime, elapsedTime);
		}
	}

	void redraw(const float currentTime, const float elapsedTime)
	{
#ifdef OS_DEBUG
		// draw each boid in flock
		iterator iter;
		for (iter = flock.begin(); iter != flock.end(); ++iter)
		{
			(*iter)->draw();
		}

		// display status in the upper left corner of the window
		std::ostringstream status;
		status << "Flock size: " << flock.size();
		status << "\n PD type: ";
		switch (cyclePD)
		{
		case 0:
			status << "LQ bin lattice";
			break;
		case 1:
			status << "brute force";
			break;
		}
///		status << "\n[F4]    Obstacles: ";
///		switch (constraint)
///		{
///		case none:
///			status << "none (wrap-around at sphere boundary)";
///			break;
///		case insideSphere:
///			status << "inside a sphere";
///			break;
///		case outsideSphere:
///			status << "inside a sphere, outside another";
///			break;
///		case outsideSpheres:
///			status << "inside a sphere, outside several";
///			break;
///		case outsideSpheresNoBig:
///			status << "outside several spheres, with wrap-around";
///			break;
///		case rectangle:
///			status << "inside a sphere, with a rectangle";
///			break;
///		case rectangleNoBig:
///			status << "a rectangle, with wrap-around";
///			break;
///		case outsideBox:
///			status << "inside a sphere, outside a box";
///			break;
///		case insideBox:
///			status << "inside a box";
///			break;
///		}
		status << std::endl;
/////		const float h = drawGetWindowHeight();
/////		const Vec3 screenLocation(10, h - 50, 0);
		const Vec3 screenLocation(-1.0, 0.9, 0);
/////		draw2dTextAt2dLocation(status, screenLocation, gGray80,
/////				drawGetWindowWidth(), drawGetWindowHeight());
		draw2dTextAt2dLocation(status, screenLocation, gGray80, 0.0, 0.0);

		///FIXME: delegated to external plugin initialization
///		drawObstacles();
#endif
	}

	void close(void)
	{
///		// delete each member of the flock
///		while (population > 0)
///			removeBoidFromFlock();

		// delete the proximity database
		delete pd;
		pd = NULL;
	}

	void reset(void)
	{
		// reset each boid in flock
		iterator iter;
		for (iter = flock.begin(); iter != flock.end(); ++iter)
		{
			(*iter)->reset();
		}
	}

	// for purposes of demonstration, allow cycling through various
	// types of proximity databases.  this routine is called when the
	// OpenSteerDemo user pushes a function key.
	void nextPD(void)
	{
		// save pointer to old PD
		ProximityDatabase* oldPD = pd;

		// allocate new PD
		const int totalPD = 2;
		switch (cyclePD = (cyclePD + 1) % totalPD)
		{
		case 0:
		{
			const Vec3 center;
			const float div = 10.0f;
			const Vec3 divisions(div, div, div);
			const float diameter = worldRadius * 1.1f * 2;
			const Vec3 dimensions(diameter, diameter, diameter);
			typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
			pd = new LQPDAV(center, dimensions, divisions);
			break;
		}
		case 1:
		{
			pd = new BruteForceProximityDatabase<AbstractVehicle*>();
			break;
		}
		}

		// switch each boid to new PD
		for (iterator i = flock.begin(); i != flock.end(); i++)
			(**i).newPD(*pd);

		// delete old PD (if any)
		delete oldPD;
	}

///	void handleFunctionKeys(int keyNumber)
///	{
///		switch (keyNumber)
///		{
///		case 1:
///			addBoidToFlock();
///			break;
///		case 2:
///			removeBoidFromFlock();
///			break;
///		case 3:
///			nextPD();
///			break;
///		case 4:
///			nextBoundaryCondition();
///			break;
///		case 5:
///			printLQbinStats();
///			break;
///		}
///	}

///	void printLQbinStats(void)
///	{
///#ifndef NO_LQ_BIN_STATS
///		int min, max;
///		float average;
///		Boid& aBoid = **(flock.begin());
///		aBoid.proximityToken->getBinPopulationStats(min, max, average);
///		std::cout << std::setprecision(2) << std::setiosflags(std::ios::fixed);
///		std::cout << "Bin populations: min, max, average: " << min << ", "
///				<< max << ", " << average << " (non-empty bins)" << std::endl;
///		std::cout << "Boid neighbors:  min, max, average: "
///				<< Boid::minNeighbors << ", " << Boid::maxNeighbors << ", "
///				<< ((float) Boid::totalNeighbors) / ((float) population)
///				<< std::endl;
///#endif // NO_LQ_BIN_STATS
///	}

///	void printMiniHelpForFunctionKeys(void)
///	{
///		std::ostringstream message;
///		message << "Function keys handled by ";
///		message << '"' << name() << '"' << ':' << std::ends;
/////		OpenSteerDemo::printMessage(message);
/////		OpenSteerDemo::printMessage("  F1     add a boid to the flock.");
/////		OpenSteerDemo::printMessage("  F2     remove a boid from the flock.");
/////		OpenSteerDemo::printMessage("  F3     use next proximity database.");
/////		OpenSteerDemo::printMessage("  F4     next flock boundary condition.");
/////		OpenSteerDemo::printMessage("");
///	}

	virtual bool addVehicle(AbstractVehicle* vehicle)
	{
		if (not PlugInAddOnMixin<OpenSteer::PlugIn>::addVehicle(vehicle))
		{
			return false;
		}
		// try to allocate a token for this boid in the proximity database
		Boid<Entity>* boid =
				dynamic_cast<Boid<Entity>*>(vehicle);
		if (boid)
		{
			//set boid world sphere (center/radius)
			boid->worldCenter = worldCenter;
			boid->worldRadius = worldRadius;
			//if not ExternalBoid then randomize
			if (not dynamic_cast<ExternalBoid<Entity>*>(boid))
			{
				// randomize initial orientation
				boid->regenerateOrthonormalBasisUF(RandomUnitVector());
				// randomize initial position: inside the world sphere
				boid->setPosition(
						worldCenter
								+ RandomVectorInUnitRadiusSphere()
										* worldRadius);
			}
			// allocate a token for this boid in the proximity database
			boid->newPD(*pd);
			// notify proximity database that our position has changed
			boid->proximityToken->updateForNewPosition(boid->position());
			//set obstacles
			boid->obstacles = obstacles;
			//set neighbors
			boid->neighbors = &neighbors;
			//set result
			return true;
		}
		//roll back addition
		PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle);
		//
		return false;
	}

///	void addBoidToFlock(void)
///	{
///		population++;
///		Boid* boid = new Boid(*pd);
///		flock.push_back(boid);
///		if (population == 1)
///		{
/////			OpenSteerDemo::selectedVehicle = boid;
///			selectedVehicle = boid;
///		}
///	}

///	void removeBoidFromFlock(void)
///	{
///		if (population > 0)
///		{
///			// save a pointer to the last boid, then remove it from the flock
///			const Boid* boid = flock.back();
///			flock.pop_back();
///			population--;
///			// if it is OpenSteerDemo's selected vehicle, unselect it
/////			if (boid == OpenSteerDemo::selectedVehicle)
/////				OpenSteerDemo::selectedVehicle = NULL;
///			if (boid == selectedVehicle)
///			{
///				selectedVehicle = NULL;
///			}
///			// delete the Boid
///			delete boid;
///		}
///	}

	// return an AVGroup containing each boid of the flock
	const AVGroup& allVehicles(void)
	{
		return (const AVGroup&) flock;
	}

	// flock: a group (STL vector) of pointers to all boids
	typename Boid<Entity>::groupType flock;
	typedef typename Boid<Entity>::groupType::const_iterator iterator;

	AVGroup neighbors;

	// pointer to database used to accelerate proximity queries
	ProximityDatabase* pd;

///	// keep track of current flock size
///	int population;

	// which of the various proximity databases is currently in use
	int cyclePD;

	float worldRadius;
	Vec3 worldCenter;

#ifdef OS_DEBUG
	void drawObstacles(void)
	{
		// draw obstacles
		ObstacleIterator iterObstacle;
		for (iterObstacle = localObstacles->begin();
				iterObstacle != localObstacles->end(); ++iterObstacle)
		{
			(*iterObstacle)->draw(false, Color(0, 0, 0), Vec3(0, 0, 0));
		}

		gDrawer3d->setTwoSided(true);
		//draw world sphere
		drawSphere(worldCenter, worldRadius, 0.66667 * worldRadius, false, Color(0, 0, 1, 1),
				true, true, Vec3::zero);
		gDrawer3d->setTwoSided(false);
	}
#endif
};

//BoidsPlugIn gBoidsPlugIn;

// ----------------------------------------------------------------------------

}// ossup namespace

#endif /* PLUGIN_BOIDS_H_ */

