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
// "Low Speed Turn" test fixture
//
// Used to evaluate vehicle response at low speed to backward-directed
// steering force
//
// 08-20-02 cwr: created 
//
//
// ----------------------------------------------------------------------------

/**
 * \file /Ely/include/Support/OpenSteerLocal/PlugIn_LowSpeedTurn.h
 *
 * \date 2016-05-13
 * \author Craig Reynolds (modified by consultit)
 */

#ifndef PLUGIN_LOWSPEEDTURN_H_
#define PLUGIN_LOWSPEEDTURN_H_

#include <iomanip>
#include <sstream>
#include <OpenSteer/Color.h>
#include <OpenSteer/PlugIn.h>
#include "common.h"

namespace ossup
{

using namespace OpenSteer;

// ----------------------------------------------------------------------------

template<typename Entity>
class LowSpeedTurn: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a flock: an STL vector of LowSpeedTurn pointers
	typedef typename std::vector<LowSpeedTurn<Entity>*> groupType;

	// constructor
	LowSpeedTurn()
	{
		reset();
		steeringSpeed = 1.0;
	}

	// reset state
	void reset(void)
	{
		// reset vehicle state
		SimpleVehicle::reset();
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();

///		// speed along Forward direction.
///		setSpeed(startSpeed);
///		// initial position along X axis
///		setPosition(startX, 0, 0);
///		// steering force clip magnitude
///		setMaxForce(0.3f);
///		// velocity  clip magnitude
///		setMaxSpeed(1.5f);
///		// for next instance: step starting location
///		startX += 2;
///		// for next instance: step speed
///		startSpeed += 0.15f;

#ifdef OS_DEBUG
		// 15 seconds and 150 points along the trail
		this->setTrailParameters(15, 150);
#endif
	}

#ifdef OS_DEBUG
	// draw into the scene
	void draw(void)
	{
		drawBasic2dCircularVehicle(*this, gGray50);
		this->drawTrail();
	}
#endif

	// per frame simulation update
	void update(const float currentTime, const float elapsedTime)
	{
		this->applySteeringForce(steering(), elapsedTime);

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// annotation
		this->annotationVelocityAcceleration();
		this->recordTrailVertex(currentTime, this->position());
#endif
	}

	// reset starting positions
///	static void resetStarts(void)
///	{
///		startX = 0;
///		startSpeed = 0;
///	}

	// constant steering force
	Vec3 steering(void)
	{
		return Vec3(1, 0, -1) * steeringSpeed;
	}

///	// for stepping the starting conditions for next vehicle
///	static float startX;
///	static 	float startSpeed;
	float steeringSpeed;
};

//LowSpeedTurn externally updated.
template<typename Entity>
class ExternalLowSpeedTurn: public LowSpeedTurn<Entity>
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
	}
};

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo

template<typename Entity>
class LowSpeedTurnPlugIn: public PlugIn
{
public:

	LowSpeedTurnPlugIn() :
			steeringSpeed(0.0)
	{
		all.clear();
	}

	// be more "nice" to avoid a compiler warning
	virtual ~LowSpeedTurnPlugIn()
	{
	}

	const char* name(void)
	{
		return "Low Speed Turn";
	}

	float selectionOrderSortKey(void)
	{
		return 0.05f;
	}

	void open(void)
	{
///		// create a given number of agents with stepped inital parameters,
///		// store pointers to them in an array.
///		LowSpeedTurn::resetStarts();
///		for (int i = 0; i < lstCount; i++)
///			all.push_back(new LowSpeedTurn);
		steeringSpeed = 1.0f;
	}

	void update(const float currentTime, const float elapsedTime)
	{
		// update, draw and annotate each agent
		iterator iter;
		for (iter = all.begin(); iter != all.end(); ++iter)
		{
			(*iter)->update(currentTime, elapsedTime);
		}
	}

	void redraw(const float currentTime, const float elapsedTime)
	{
#ifdef OS_DEBUG
		// update, draw and annotate each agent
		iterator iter;
		for (iter = all.begin(); iter != all.end(); ++iter)
		{
			// draw this agent
			LowSpeedTurn<Entity>& agent = **iter;
			agent.draw();

			// display speed near agent's screen position
			const Color textColor(0.8f, 0.8f, 1.0f);
			const Vec3 textOffset(0, 0.25f, 0);
			const Vec3 textPosition = agent.position() + textOffset;
			std::ostringstream annote;
			annote << std::setprecision(2) << std::setiosflags(std::ios::fixed)
					<< agent.speed() << std::ends;
/////			draw2dTextAt3dLocation(annote, textPosition, textColor,
/////					drawGetWindowWidth(), drawGetWindowHeight());
			draw2dTextAt3dLocation(annote, textPosition, textColor, 1.0, 1.0);
		}
#endif
	}

	void close(void)
	{
///		for (iterator i = all.begin(); i != all.end(); i++)
///			delete (*i);
///		all.clear();
	}

	void reset(void)
	{
		// reset each agent
///		LowSpeedTurn::resetStarts();
		iterator iter;
		for (iter = all.begin(); iter != all.end(); ++iter)
		{
			(*iter)->reset();
		}
	}

	virtual bool addVehicle(AbstractVehicle* vehicle)
	{
		if (! PlugInAddOnMixin<OpenSteer::PlugIn>::addVehicle(vehicle))
		{
			return false;
		}
		// get low speed turn
		LowSpeedTurn<Entity>* lowSpeedTurn =
				dynamic_cast<LowSpeedTurn<Entity>*>(vehicle);
		if (lowSpeedTurn)
		{
			//set steering speed
			lowSpeedTurn->steeringSpeed = steeringSpeed;
			//set result
			return true;
		}
		//roll back addition
		PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle);
		//
		return false;
	}

	const AVGroup& allVehicles(void)
	{
		return (const AVGroup&) all;
	}

	typename LowSpeedTurn<Entity>::groupType all; // for allVehicles
	typedef typename LowSpeedTurn<Entity>::groupType::const_iterator iterator;

	float steeringSpeed;///serializable
};

//LowSpeedTurnPlugIn gLowSpeedTurnPlugIn;

} // ossup namespace

#endif /* PLUGIN_LOWSPEEDTURN_H_ */
