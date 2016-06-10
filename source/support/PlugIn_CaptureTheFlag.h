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
// Capture the Flag   (a portion of the traditional game)
//
// The "Capture the Flag" sample steering problem, proposed by Marcin
// Chady of the Working Group on Steering of the IGDA's AI Interface
// Standards Committee (http://www.igda.org/Committees/ai.htm) in this
// message (http://sourceforge.net/forum/message.php?msg_id=1642243):
//
//     "An agent is trying to reach a physical location while trying
//     to stay clear of a group of enemies who are activossup seeking
//     him. The environment is littered with obstacles, so collision
//     avoidance is also necessary."
//
// Note that the enemies do not make use of their knowledge of the 
// seeker's goal by "guarding" it.  
//
// XXX hmm, rename them "attacker" and "defender"?
//
// 08-12-02 cwr: created 
//
//
// ----------------------------------------------------------------------------

/**
 * \file /Ely/include/Support/OpenSteerLocal/PlugIn_CaptureTheFlag.h
 *
 * \date 2016-05-13
 * \author Craig Reynolds (modified by consultit)
 */

#ifndef PLUGIN_CAPTURETHEFLAG_H_
#define PLUGIN_CAPTURETHEFLAG_H_

#include <iomanip>
#include <string>
#include <sstream>
#include <OpenSteer/Color.h>
#include <OpenSteer/PlugIn.h>
#include "common.h"

namespace ossup
{

using namespace OpenSteer;

// ----------------------------------------------------------------------------
// globals
// (perhaps these should be member variables of a Vehicle or PlugIn class)
///const Color evadeColor(0.6f, 0.6f, 0.3f); // annotation
///const Color seekColor(0.3f, 0.6f, 0.6f); // annotation
///const Color clearPathColor(0.3f, 0.6f, 0.3f); // annotation
///const Vec3 gHomeBaseCenter(0, 0, 0);
///const float gHomeBaseRadius = 1.5;
///const float gMinStartRadius = 30;
///const float gMaxStartRadius = 40;
///const float gBrakingRate = 0.75;
///const float gAvoidancePredictTimeMin = 0.9f;
///const float gAvoidancePredictTimeMax = 2;
///float gAvoidancePredictTime = gAvoidancePredictTimeMin;
///CtfSeeker* gSeeker = NULL;
///int resetCount = 0;
// ----------------------------------------------------------------------------
// state for OpenSteerDemo PlugIn
//
// consider moving this inside CtfPlugIn
// consider using STL (any advantage? consistency?)
///CtfSeeker* ctfSeeker;
///const int ctfEnemyCount = 4;
///CtfEnemy* ctfEnemies[ctfEnemyCount];

struct CtfPlugInData
{
	Vec3 gHomeBaseCenter; //Vec3(0, 0, 0)
	float gHomeBaseRadius; //1.5
	float gMinStartRadius; //30.0
	float gMaxStartRadius; //40.0
	float gBrakingRate; //0.75
	float gAvoidancePredictTimeMin; //0.9
	float gAvoidancePredictTimeMax; //2.0 (>=gAvoidancePredictTimeMin)
	float gAvoidancePredictTime; //0.9 (=gAvoidancePredictTimeMin)
	int resetCount;
	bool gDelayedResetPlugInXXX;
#ifdef OS_DEBUG
	Color evadeColor; //Color(0.6f, 0.6f, 0.3f)
	Color seekColor; //Color(0.3f, 0.6f, 0.6f)
	Color clearPathColor; //Color(0.3f, 0.6f, 0.3f)
#endif
};

// ----------------------------------------------------------------------------
// This PlugIn uses two vehicle types: CtfSeeker and CtfEnemy.  They have a
// common base class: CtfBase which is a specialization of SimpleVehicle.

template<typename Entity>
class CtfBase: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a flock: an STL vector of CtfBase pointers
	typedef std::vector<CtfBase<Entity>*> groupType;

	// constructor
	CtfBase()
	{
		reset();
		gCtfPlugInData = NULL;
		allObstacles = NULL;
	}

	// reset state
	void reset(void)
	{
		SimpleVehicle::reset();  // reset the vehicle
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();

///		setSpeed(3);             // speed along Forward direction.
///		setMaxForce(3.0);        // steering force is clipped to this magnitude
///		setMaxSpeed(3.0);        // velocity is clipped to this magnitude

		avoiding = false;         // not activossup avoiding

///		randomizeStartingPositionAndHeading();  // new starting position

#ifdef OS_DEBUG
		this->clearTrailHistory();     // prevent long streaks due to teleportation
#endif
	}

#ifdef OS_DEBUG
	// draw this character/vehicle into the scene
	void draw(void)
	{
		drawBasic2dCircularVehicle(*this, bodyColor);
		this->drawTrail();
	}
#endif

	// annotate when activossup avoiding obstacles
	// xxx perhaps this should be a call to a general purpose annotation
	// xxx for "local xxx axis aligned box in XZ plane" -- same code in in
	// xxx Pedestrian.cpp
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

///	void drawHomeBase(void);

	void randomizeStartingPositionAndHeading(void)
	{
///		// randomize position on a ring between inner and outer radii
///		// centered around the home base
///		const float rRadius = frandom2(gCtfPlugInData->gMinStartRadius, m_CtfPlugInData->gMaxStartRadius);
///		const Vec3 randomOnRing = RandomUnitVectorOnXZPlane() * rRadius;
///		this->setPosition(gCtfPlugInData->gHomeBaseCenter + randomOnRing);
///		// are we are too close to an obstacle?
///		if (minDistanceToObstacle(this->position()) < this->radius() * 5)
///		{
///			// if so, retry the randomization (this recursive call may not return
///			// if there is too little free space)
///			randomizeStartingPositionAndHeading();
///		}
///		else
///		{
		// otherwise, if the position is OK, randomize 2D heading
		this->randomizeHeadingOnXZPlane();
///		}
	}

	enum seekerState
	{
		running, tagged, atGoal
	};

	// for draw method
	Color bodyColor;

	// xxx store steer sub-state for anotation
	bool avoiding;

	CtfPlugInData* gCtfPlugInData;
	ObstacleGroup* allObstacles;
};

//enemy declarations
template<typename Entity> class CtfEnemy;

template<typename Entity>
class CtfSeeker: public CtfBase<Entity>
{
public:

	// constructor
	CtfSeeker()
	{
		reset();
		gCtfEnemies = NULL;
	}

	// reset state
	void reset(void)
	{
		CtfBase<Entity>::reset();
		this->bodyColor.set(0.4f, 0.4f, 0.6f); // blueish
///		gSeeker = this;
		state = CtfBase<Entity>::running;
		evading = false;
	}

	// update method for goal seeker
	// per frame simulation update
	void update(const float currentTime, const float elapsedTime)
	{
		// do behavioral state transitions, as needed
		updateState(currentTime);

		// determine and apply steering/braking forces
		Vec3 steer(0, 0, 0);
		if (state == CtfBase<Entity>::running)
		{
			steer = steeringForSeeker();
		}
		else
		{
			this->applyBrakingForce(this->gCtfPlugInData->gBrakingRate,
					elapsedTime);
		}
		this->applySteeringForce(steer, elapsedTime);

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// annotation
		this->annotationVelocityAcceleration();
		this->recordTrailVertex(currentTime, this->position());
#endif
	}

	bool clearPathToGoal(void);

	Vec3 steeringForSeeker(void)
	{
		// determine if obstacle avodiance is needed
		const bool clearPath = clearPathToGoal();
		adjustObstacleAvoidanceLookAhead(clearPath);
		const Vec3 obstacleAvoidance = this->steerToAvoidObstacles(
				this->gCtfPlugInData->gAvoidancePredictTime,
				*(this->allObstacles));

		// saved for annotation
		this->avoiding = (obstacleAvoidance != Vec3::zero);

		if (this->avoiding)
		{
			// use pure obstacle avoidance if needed
			return obstacleAvoidance;
		}
		else
		{
			// otherwise seek home base and perhaps evade defenders
			const Vec3 seek = this->xxxsteerForSeek(
					this->gCtfPlugInData->gHomeBaseCenter);
			if (clearPath)
			{
				// we have a clear path (defender-free corridor), use pure seek

				// xxx experiment 9-16-02
				Vec3 s = limitMaxDeviationAngle(seek, 0.707f, this->forward());

#ifdef OS_DEBUG
				this->annotationLine(this->position(),
						this->position() + (s * 0.2f),
						this->gCtfPlugInData->seekColor);
#endif
				return s;
			}
			else
			{
				if (0) // xxx testing new evade code xxx
				{
					// combine seek and (forward facing portion of) evasion
					const Vec3 evade = steerToEvadeAllDefenders();
					const Vec3 steer = seek
							+ limitMaxDeviationAngle(evade, 0.5f,
									this->forward());

#ifdef OS_DEBUG
					// annotation: show evasion steering force
					this->annotationLine(this->position(),
							this->position() + (steer * 0.2f),
							this->gCtfPlugInData->evadeColor);
#endif
					return steer;
				}
				else

				{
					const Vec3 evade = XXXsteerToEvadeAllDefenders();
					const Vec3 steer = limitMaxDeviationAngle(seek + evade,
							0.707f, this->forward());

#ifdef OS_DEBUG
					this->annotationLine(this->position(),
							this->position() + seek, gRed);
					this->annotationLine(this->position(),
							this->position() + evade, gGreen);

					// annotation: show evasion steering force
					this->annotationLine(this->position(),
							this->position() + (steer * 0.2f),
							this->gCtfPlugInData->evadeColor);
#endif
					return steer;
				}
			}
		}
	}

	void updateState(const float currentTime)
	{
		// if we reach the goal before being tagged, switch to atGoal state
		if (state == CtfBase<Entity>::running)
		{
			const float baseDistance = Vec3::distance(this->position(),
					this->gCtfPlugInData->gHomeBaseCenter);
			if (baseDistance
					< (this->radius() + this->gCtfPlugInData->gHomeBaseRadius))
				state = CtfBase<Entity>::atGoal;
		}

		// update lastRunningTime (holds off reset time)
		if (state == CtfBase<Entity>::running)
		{
			lastRunningTime = currentTime;
		}
		else
		{
			const float resetDelay = 4;
			const float resetTime = lastRunningTime + resetDelay;
			if (currentTime > resetTime)
			{
				// xxx a royal hack (should do this internal to CTF):
				this->gCtfPlugInData->gDelayedResetPlugInXXX = true;
			}
		}
	}

#ifdef OS_DEBUG
	void draw(void)
	{
		// first call the draw method in the base class
		CtfBase<Entity>::draw();

		// select string describing current seeker state
		std::string seekerStateString("");
		switch (state)
		{
		case CtfBase<Entity>::running:
			if (this->avoiding)
				seekerStateString = "avoid obstacle";
			else if (evading)
				seekerStateString = "seek and evade";
			else
				seekerStateString = "seek goal";
			break;
		case CtfBase<Entity>::tagged:
			seekerStateString = "tagged";
			break;
		case CtfBase<Entity>::atGoal:
			seekerStateString = "reached goal";
			break;
		}

		// annote seeker with its state as text
		const Vec3 textOrigin = this->position() + Vec3(0, 0.25, 0);
		std::ostringstream annote;
		annote << seekerStateString << std::endl;
		annote << std::setprecision(2) << std::setiosflags(std::ios::fixed)
				<< this->speed() << std::ends;
/////	draw2dTextAt3dLocation(annote, textOrigin, gWhite, drawGetWindowWidth(),
/////			drawGetWindowHeight());
		draw2dTextAt3dLocation(annote, textOrigin, gWhite, 0.0, 0.0);

		// display status in the upper left corner of the window
		std::ostringstream status;
		status << seekerStateString << std::endl;
///		status << obstacleCount << " obstacles [F1/F2]" << std::endl;
		status << this->gCtfPlugInData->resetCount << " restarts" << std::ends;
///	//	const float h = drawGetWindowHeight();
///	//	const Vec3 screenLocation(10, h - 50, 0);
		const Vec3 screenLocation(-1.0, 0.9, 0);
///	//	draw2dTextAt2dLocation(status, screenLocation, gGray80,
///	//			drawGetWindowWidth(), drawGetWindowHeight());
		draw2dTextAt2dLocation(status, screenLocation, gGray80, 0.0, 0.0);
	}
#endif

	Vec3 steerToEvadeAllDefenders(void);

	Vec3 XXXsteerToEvadeAllDefenders(void);

	// adjust obstacle avoidance look ahead time: make it large when we are far
	// from the goal and heading directly towards it, make it small otherwise.
	void adjustObstacleAvoidanceLookAhead(const bool clearPath)
	{
		if (clearPath)
		{
			evading = false;
			const float goalDistance = Vec3::distance(
					this->gCtfPlugInData->gHomeBaseCenter, this->position());
			const bool headingTowardGoal = this->isAhead(
					this->gCtfPlugInData->gHomeBaseCenter, 0.98f);
			const bool isNear = (goalDistance / this->speed())
					< this->gCtfPlugInData->gAvoidancePredictTimeMax;
			const bool useMax = headingTowardGoal && !isNear;
			this->gCtfPlugInData->gAvoidancePredictTime = (
					useMax ?
							this->gCtfPlugInData->gAvoidancePredictTimeMax :
							this->gCtfPlugInData->gAvoidancePredictTimeMin);
		}
		else
		{
			evading = true;
			this->gCtfPlugInData->gAvoidancePredictTime =
					this->gCtfPlugInData->gAvoidancePredictTimeMin;
		}
	}

#ifdef OS_DEBUG
	void clearPathAnnotation(const float sideThreshold,
			const float behindThreshold, const Vec3& goalDirection)
	{
		const Vec3 behindSide = this->side() * sideThreshold;
		const Vec3 behindBack = this->forward() * -behindThreshold;
		const Vec3 pbb = this->position() + behindBack;
		const Vec3 gun = this->localRotateForwardToSide(goalDirection);
		const Vec3 gn = gun * sideThreshold;
		const Vec3 hbc = this->gCtfPlugInData->gHomeBaseCenter;
		this->annotationLine(pbb + gn, hbc + gn,
				this->gCtfPlugInData->clearPathColor);
		this->annotationLine(pbb - gn, hbc - gn,
				this->gCtfPlugInData->clearPathColor);
		this->annotationLine(hbc - gn, hbc + gn,
				this->gCtfPlugInData->clearPathColor);
		this->annotationLine(pbb - behindSide, pbb + behindSide,
				this->gCtfPlugInData->clearPathColor);
	}
#endif

	typename CtfBase<Entity>::seekerState state;
	bool evading; // xxx store steer sub-state for anotation
	float lastRunningTime; // for auto-reset

	std::vector<CtfEnemy<Entity>*>* gCtfEnemies;

};

//CtfSeeker externally updated.
template<typename Entity>
class ExternalCtfSeeker: public CtfSeeker<Entity>
{
public:
	void update(const float currentTime, const float elapsedTime)
	{
		// do behavioral state transitions, as needed
		this->updateState(currentTime);

		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// annotation
		this->annotationVelocityAcceleration();
		this->recordTrailVertex(currentTime, this->position());
#endif
	}
};

template<typename Entity>
class CtfEnemy: public CtfBase<Entity>
{
public:

	// type for a player: an STL vector of Player pointers
	typedef typename std::vector<CtfEnemy<Entity>*> groupType;

	// constructor
	CtfEnemy()
	{
		reset();
		gSeeker = NULL;
	}

	// reset state
	void reset(void)
	{
		CtfBase<Entity>::reset();
		this->bodyColor.set(0.6f, 0.4f, 0.4f); // redish
	}

	// per frame simulation update
	void update(const float currentTime, const float elapsedTime)
	{
		if (not gSeeker)
		{
			return;
		}

		// determine upper bound for pursuit prediction time
		const float seekerToGoalDist = Vec3::distance(
				this->gCtfPlugInData->gHomeBaseCenter, gSeeker->position());
		const float adjustedDistance = seekerToGoalDist - this->radius()
				- this->gCtfPlugInData->gHomeBaseRadius;
		const float seekerToGoalTime = (
				(adjustedDistance < 0) ?
						0 : (adjustedDistance / gSeeker->speed()));
		const float maxPredictionTime = seekerToGoalTime * 0.9f;

		// determine steering (pursuit, obstacle avoidance, or braking)
		Vec3 steer(0, 0, 0);
		if (gSeeker->state == CtfBase<Entity>::running)
		{
			const Vec3 avoidance = this->steerToAvoidObstacles(
					this->gCtfPlugInData->gAvoidancePredictTimeMin,
					*(this->allObstacles));

			// saved for annotation
			this->avoiding = (avoidance == Vec3::zero);

			if (this->avoiding)
				steer = this->steerForPursuit(*gSeeker, maxPredictionTime);
			else
				steer = avoidance;
		}
		else
		{
			this->applyBrakingForce(this->gCtfPlugInData->gBrakingRate,
					elapsedTime);
		}
		this->applySteeringForce(steer, elapsedTime);

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// annotation
		this->annotationVelocityAcceleration();
		this->recordTrailVertex(currentTime, this->position());
#endif

		// detect and record interceptions ("tags") of seeker
		const float seekerToMeDist = Vec3::distance(this->position(),
				gSeeker->position());
		const float sumOfRadii = this->radius() + gSeeker->radius();
		if (seekerToMeDist < sumOfRadii)
		{
			if (gSeeker->state == CtfBase<Entity>::running)
				gSeeker->state = CtfBase<Entity>::tagged;

#ifdef OS_DEBUG
			// annotation:
			if (gSeeker->state == CtfBase<Entity>::tagged)
			{
				const Color color(0.8f, 0.5f, 0.5f);
				this->annotationXZDisk(sumOfRadii,
						(this->position() + gSeeker->position()) / 2, color,
						20);
			}
#endif
		}
	}

	CtfSeeker<Entity>* gSeeker;
};

//CtfEnemy externally updated.
template<typename Entity>
class ExternalCtfEnemy: public CtfEnemy<Entity>
{
public:
	void update(const float currentTime, const float elapsedTime)
	{
		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// annotation
		this->annotationVelocityAcceleration();
		this->recordTrailVertex(currentTime, this->position());
#endif

		// detect and record interceptions ("tags") of seeker
		const float seekerToMeDist = Vec3::distance(this->position(),
				this->gSeeker->position());
		const float sumOfRadii = this->radius() + this->gSeeker->radius();
		if (seekerToMeDist < sumOfRadii)
		{
			if (this->gSeeker->state == CtfBase<Entity>::running)
				this->gSeeker->state = CtfBase<Entity>::tagged;

#ifdef OS_DEBUG
			// annotation:
			if (this->gSeeker->state == CtfBase<Entity>::tagged)
			{
				const Color color(0.8f, 0.5f, 0.5f);
				this->annotationXZDisk(sumOfRadii,
						(this->position() + this->gSeeker->position()) / 2,
						color, 20);
			}
#endif
		}
	}
};

#define testOneObstacleOverlap(radius, center)               \
    {                                                            \
        float d = Vec3::distance (c, center);                    \
        float clearance = d - (r + (radius));                    \
        if (minClearance > clearance) minClearance = clearance;  \
    }

/// ----------------------------------------------------------------------------
///external definitions due to cross references

// are there any enemies along the corridor between us and the goal?
// is there a clear path to the goal?
template<typename Entity> inline bool CtfSeeker<Entity>::clearPathToGoal(void)
{
	const float sideThreshold = this->radius() * 8.0f;
	const float behindThreshold = this->radius() * 2.0f;

	const Vec3 goalOffset = this->gCtfPlugInData->gHomeBaseCenter
			- this->position();
	const float goalDistance = goalOffset.length();
	const Vec3 goalDirection = goalOffset / goalDistance;

	const bool goalIsAside = this->isAside(
			this->gCtfPlugInData->gHomeBaseCenter, 0.5);

	// for annotation: loop over all and save result, instead of early return
	bool xxxReturn = true;

	// loop over enemies
	typename std::vector<CtfEnemy<Entity>*>::iterator iter;
	for (iter = gCtfEnemies->begin(); iter != gCtfEnemies->end(); ++iter)
	{
		// short name for this enemy
		const CtfEnemy<Entity>& e = *(*iter);
		const float eDistance = Vec3::distance(this->position(), e.position());
		const float timeEstimate = 0.3f * eDistance / e.speed(); //xxx
		const Vec3 eFuture = e.predictFuturePosition(timeEstimate);
		const Vec3 eOffset = eFuture - this->position();
		const float alongCorridor = goalDirection.dot(eOffset);
		const bool inCorridor = ((alongCorridor > -behindThreshold)
				&& (alongCorridor < goalDistance));
		const float eForwardDistance = this->forward().dot(eOffset);

#ifdef OS_DEBUG
		// xxx temp move this up before the conditionals
		this->annotationXZCircle(e.radius(), eFuture,
				this->gCtfPlugInData->clearPathColor, 20); //xxx
#endif

		// consider as potential blocker if within the corridor
		if (inCorridor)
		{
			const Vec3 perp = eOffset - (goalDirection * alongCorridor);
			const float acrossCorridor = perp.length();
			if (acrossCorridor < sideThreshold)
			{
				// not a blocker if behind us and we are perp to corridor
				const float eFront = eForwardDistance + e.radius();

#ifdef OS_DEBUG
				//annotationLine (position, forward*eFront, gGreen); // xxx
				//annotationLine (e.position, forward*eFront, gGreen); // xxx

				// xxx
				// std::ostringstream message;
				// message << "eFront = " << std::setprecision(2)
				//         << std::setiosflags(std::ios::fixed) << eFront << std::ends;
				// draw2dTextAt3dLocation (*message.str(), eFuture, gWhite);
#endif

				const bool eIsBehind = eFront < -behindThreshold;
				const bool eIsWayBehind = eFront < (-2 * behindThreshold);
				const bool safeToTurnTowardsGoal = ((eIsBehind && goalIsAside)
						|| eIsWayBehind);

				if (!safeToTurnTowardsGoal)
				{
					// this enemy blocks the path to the goal, so return false
#ifdef OS_DEBUG
					this->annotationLine(this->position(), e.position(),
							this->gCtfPlugInData->clearPathColor);
#endif
					// return false;
					xxxReturn = false;
				}
			}
		}
	}

	// no enemies found along path, return true to indicate path is clear
	// clearPathAnnotation (sideThreshold, behindThreshold, goalDirection);
	// return true;
	//if (xxxReturn)
#ifdef OS_DEBUG
	clearPathAnnotation(sideThreshold, behindThreshold, goalDirection);
#endif
	return xxxReturn;
}

template<typename Entity> inline Vec3 CtfSeeker<Entity>::steerToEvadeAllDefenders(void)
{
	Vec3 evade(0, 0, 0);
	const float goalDistance = Vec3::distance(
			this->gCtfPlugInData->gHomeBaseCenter, this->position());

	// sum up weighted evasion
	typename std::vector<CtfEnemy<Entity>*>::iterator iter;
	for (iter = gCtfEnemies->begin(); iter != gCtfEnemies->end(); ++iter)
	{
		const CtfEnemy<Entity>& e = *(*iter);
		const Vec3 eOffset = e.position() - this->position();
		const float eDistance = eOffset.length();

		const float eForwardDistance = this->forward().dot(eOffset);
		const float behindThreshold = this->radius() * 2;
		const bool behind = eForwardDistance < behindThreshold;
		if ((!behind) || (eDistance < 5))
		{
			if (eDistance < (goalDistance * 1.2)) //xxx
			{
				// const float timeEstimate = 0.5f * eDistance / e.speed;//xxx
				const float timeEstimate = 0.15f * eDistance / e.speed(); //xxx
				const Vec3 future = e.predictFuturePosition(timeEstimate);

#ifdef OS_DEBUG
				this->annotationXZCircle(e.radius(), future,
						this->gCtfPlugInData->evadeColor, 20); // xxx
#endif

				const Vec3 offset = future - this->position();
				const Vec3 lateral = offset.perpendicularComponent(
						this->forward());
				const float d = lateral.length();
				const float weight = -1000 / (d * d);
				evade += (lateral / d) * weight;
			}
		}
	}
	return evade;
}

template<typename Entity> inline Vec3 CtfSeeker<Entity>::XXXsteerToEvadeAllDefenders(void)
{
	// sum up weighted evasion
	Vec3 evade(0, 0, 0);
	typename std::vector<CtfEnemy<Entity>*>::iterator iter;
	for (iter = gCtfEnemies->begin(); iter != gCtfEnemies->end(); ++iter)
	{
		const CtfEnemy<Entity>& e = *(*iter);
		const Vec3 eOffset = e.position() - this->position();
		const float eDistance = eOffset.length();

		// xxx maybe this should take into account e's heading? xxx
		const float timeEstimate = 0.5f * eDistance / e.speed(); //xxx
		const Vec3 eFuture = e.predictFuturePosition(timeEstimate);

#ifdef OS_DEBUG
		// annotation
		this->annotationXZCircle(e.radius(), eFuture,
				this->gCtfPlugInData->evadeColor, 20);
#endif

		// steering to flee from eFuture (enemy's future position)
		const Vec3 flee = this->xxxsteerForFlee(eFuture);

		const float eForwardDistance = this->forward().dot(eOffset);
		const float behindThreshold = this->radius() * -2;

		const float distanceWeight = 4 / eDistance;
		const float forwardWeight = (
				(eForwardDistance > behindThreshold) ? 1.0f : 0.5f);

		const Vec3 adjustedFlee = flee * distanceWeight * forwardWeight;

		evade += adjustedFlee;
	}
	return evade;
}

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo

template<typename Entity>
class CtfPlugIn: public PlugIn
{
public:

	const char* name(void)
	{
		return "Capture the Flag";
	}

	float selectionOrderSortKey(void)
	{
		return 0.01f;
	}

	virtual ~CtfPlugIn()
	{
	} // be more "nice" to avoid a compiler warning

	void open(void)
	{
		m_CtfPlugInData.gHomeBaseCenter = Vec3(0, 0, 0);
		m_CtfPlugInData.gHomeBaseRadius = 1.5;
		m_CtfPlugInData.gMinStartRadius = 30;
		m_CtfPlugInData.gMaxStartRadius = 40;
		m_CtfPlugInData.gBrakingRate = 0.75;
		m_CtfPlugInData.gAvoidancePredictTimeMin = 0.9f;
		m_CtfPlugInData.gAvoidancePredictTimeMax = 2;
		m_CtfPlugInData.gAvoidancePredictTime = m_CtfPlugInData.gAvoidancePredictTimeMin;
		m_CtfPlugInData.resetCount = 0;
		m_CtfPlugInData.gDelayedResetPlugInXXX = false;
#ifdef OS_DEBUG
		m_CtfPlugInData.evadeColor = Color(0.6f, 0.6f, 0.3f); // annotation
		m_CtfPlugInData.seekColor = Color(0.3f, 0.6f, 0.6f); // annotation
		m_CtfPlugInData.clearPathColor = Color(0.3f, 0.6f, 0.3f); // annotation
#endif

///		// create the seeker ("hero"/"attacker")
///		ctfSeeker = new CtfSeeker;
		ctfSeeker = NULL;
///		all.push_back(ctfSeeker);
///		// create the specified number of enemies,
///		// storing pointers to them in an array.
///		for (int i = 0; i < ctfEnemyCount; i++)
///		{
///			ctfEnemies[i] = new CtfEnemy;
///			all.push_back(ctfEnemies[i]);
///		}
	}

	void update(const float currentTime, const float elapsedTime)
	{
		// service queued reset request, if any
		if (m_CtfPlugInData.gDelayedResetPlugInXXX)
		{
			reset();
			m_CtfPlugInData.gDelayedResetPlugInXXX = false;
		}

///		// update the seeker
///		ctfSeeker->update(currentTime, elapsedTime);
///		// update each enemy
///		for (int i = 0; i < ctfEnemyCount; i++)
///		{
///			ctfEnemies[i]->update(currentTime, elapsedTime);
///		}
		// update each vehicles
		iterator iter;
		for (iter = all.begin(); iter != all.end(); ++iter)
		{
			(*iter)->update(currentTime, elapsedTime);
		}
	}

	void redraw(const float currentTime, const float elapsedTime)
	{
#ifdef OS_DEBUG
		// draw each vehicles
		iterator iter;
		for (iter = all.begin(); iter != all.end(); ++iter)
		{
			(*iter)->draw();
		}
		// draw the seeker, obstacles and home base
///		ctfSeeker->draw();

		///FIXME: delegated to external plugin initialization
///		drawObstacles();
///		drawHomeBase();

///		// draw each enemy
///		for (int i = 0; i < ctfEnemyCount; i++)
///			ctfEnemies[i]->draw();
#endif
	}

	void close(void)
	{
///		// delete seeker
///		delete (ctfSeeker);
///		ctfSeeker = NULL;
///		// delete each enemy
///		for (int i = 0; i < ctfEnemyCount; i++)
///		{
///			delete (ctfEnemies[i]);
///			ctfEnemies[i] = NULL;
///		}
///		// clear the group of all vehicles
///		all.clear();
	}

	void reset(void)
	{
		// reset each vehicles
		iterator iter;
		for (iter = all.begin(); iter != all.end(); ++iter)
		{
			(*iter)->reset();
		}
	}

///	void handleFunctionKeys(int keyNumber)
///	{
///		switch (keyNumber)
///		{
///		case 1:
///			CtfBase::addOneObstacle();
///			break;
///		case 2:
///			CtfBase::removeOneObstacle();
///			break;
///		}
///	}

///	void printMiniHelpForFunctionKeys(void)
///	{
///		std::ostringstream message;
///		message << "Function keys handled by ";
///		message << '"' << name() << '"' << ':' << std::ends;
/////		OpenSteerDemo::printMessage(message);
/////		OpenSteerDemo::printMessage("  F1     add one obstacle.");
/////		OpenSteerDemo::printMessage("  F2     remove one obstacle.");
/////		OpenSteerDemo::printMessage("");
///	}

	virtual bool addVehicle(AbstractVehicle* vehicle)
	{
		if (not PlugInAddOnMixin<OpenSteer::PlugIn>::addVehicle(vehicle))
		{
			return false;
		}
		//check if this is a CtfSeeker
		CtfSeeker<Entity>* ctfSeekerTmp =
				dynamic_cast<CtfSeeker<Entity>*>(vehicle);
		if (ctfSeekerTmp)
		{
			//if not ExternalCtfSeeker then randomize
			if (not dynamic_cast<ExternalCtfSeeker<Entity>*>(ctfSeekerTmp))
			{
				// randomize 2D heading
				ctfSeekerTmp->randomizeStartingPositionAndHeading();
			}
			// set seeker plugin data & obstacles
			ctfSeekerTmp->gCtfPlugInData = &m_CtfPlugInData;
			ctfSeekerTmp->allObstacles = obstacles;
			// set seeker enemies
			ctfSeekerTmp->gCtfEnemies = &ctfEnemies;
			// set the plugin's seeker: the last added one
			ctfSeeker = ctfSeekerTmp;
			//update each enemy's seeker
			setAllEnemiesSeeker();
			//that's all
			return true;
		}
		//or if this is a CtfEnemy
		CtfEnemy<Entity>* ctfEnemyTmp =
			dynamic_cast<CtfEnemy<Entity>*>(vehicle);
		if (ctfEnemyTmp)
		{
			//if not ExternalCtfEnemy then randomize
			if (not dynamic_cast<ExternalCtfEnemy<Entity>*>(ctfEnemyTmp))
			{
				// randomize only 2D heading
				ctfEnemyTmp->randomizeStartingPositionAndHeading();
			}
			// set enemy plugin data & obstacles
			ctfEnemyTmp->gCtfPlugInData = &m_CtfPlugInData;
			ctfEnemyTmp->allObstacles = obstacles;
			// set the enemy's seeker
			ctfEnemyTmp->gSeeker = ctfSeeker;
			// add enemy to enemy repo
			ctfEnemies.push_back(ctfEnemyTmp);
			//that's all
			return true;
		}
		//roll back addition
		PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle);
		//
		return false;
	}

	virtual bool removeVehicle(OpenSteer::AbstractVehicle* vehicle)
	{
		if (not PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle))
		{
			return false;
		}
		//check if this is the current seeker
		if (vehicle == ctfSeeker)
		{
			ctfSeeker = NULL;
			// find a new seeker (if any): the first found or NULL
			iterator iter;
			for (iter = all.begin(); iter != all.end(); ++iter)
			{
				ctfSeeker = dynamic_cast<CtfSeeker<Entity>*>(*iter);
				if (ctfSeeker)
				{
					break;
				}
			}
			//update each enemy's seeker
			setAllEnemiesSeeker();
		}
		//check if this is a CtfEnemy
		CtfEnemy<Entity>* ctfEnemyTmp = dynamic_cast<CtfEnemy<Entity>*>(vehicle);
		if (ctfEnemyTmp)
		{
			//remove it also from enemy repo
			typename CtfEnemy<Entity>::groupType::iterator iter;
			for (iter = ctfEnemies.begin(); iter != ctfEnemies.end(); ++iter)
			{
				if (*iter == ctfEnemyTmp)
				{
					ctfEnemies.erase(iter);
					break;
				}
			}
		}
		//
		return true;
	}

	void setAllEnemiesSeeker()
	{
		typename CtfEnemy<Entity>::groupType::iterator iter;
		for (iter = ctfEnemies.begin(); iter != ctfEnemies.end(); ++iter)
		{
			// update the enemy's seeker
			(*iter)->gSeeker = ctfSeeker;
		}
	}

	const AVGroup& allVehicles(void)
	{
		return (const AVGroup&) all;
	}

#ifdef OS_DEBUG
	void drawHomeBase(void)
	{
		const Vec3 up(0, 0.01f, 0);
		const Color atColor(0.3f, 0.3f, 0.5f);
		const Color noColor = gGray50;
		bool reached;
		if(ctfSeeker)
		{
			reached = ctfSeeker->state == CtfSeeker<Entity>::atGoal;
		}
		else
		{
			reached = false;
		}
		const Color baseColor = (reached ? atColor : noColor);
		drawXZDisk(m_CtfPlugInData.gHomeBaseRadius, m_CtfPlugInData.gHomeBaseCenter, baseColor, 40);
		drawXZDisk(m_CtfPlugInData.gHomeBaseRadius / 15, m_CtfPlugInData.gHomeBaseCenter + up, gBlack, 20);
	}

	void drawObstacles(void)
	{
		const Color color(0.8f, 0.6f, 0.4f);
///		const SOG& allSO = CtfBase::allObstacles;
///		for (SOI so = allSO.begin(); so != allSO.end(); so++)
///		{
///			drawXZCircle((**so).radius, (**so).center, color, 40);
///		}
		// draw obstacles
		ObstacleIterator iterObstacle;
		for (iterObstacle = localObstacles->begin();
				iterObstacle != localObstacles->end(); ++iterObstacle)
		{
			(*iterObstacle)->draw(false, color, Vec3(0, 0, 0));
		}
	}
#endif

	// a group (STL vector) of all vehicles in the PlugIn
	typename CtfBase<Entity>::groupType all;
	typedef typename CtfBase<Entity>::groupType::const_iterator iterator;
	typename CtfEnemy<Entity>::groupType ctfEnemies;
	CtfSeeker<Entity>* ctfSeeker;

	CtfPlugInData m_CtfPlugInData;
};

//CtfPlugIn gCtfPlugIn;

} // ossup namespace

#endif /* PLUGIN_CAPTURETHEFLAG_H_ */
