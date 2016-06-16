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
// Multiple pursuit (for testing pursuit)
//
// 08-22-02 cwr: created 
//
//
// ----------------------------------------------------------------------------
/**
 * \file /Ely/include/Support/OpenSteerLocal/PlugIn_MultiplePursuit.h
 *
 * \date 2016-05-13
 * \author Craig Reynolds (modified by consultit)
 */

#ifndef PLUGIN_MULTIPLEPURSUIT_H_
#define PLUGIN_MULTIPLEPURSUIT_H_

#include <OpenSteer/Color.h>
#include <OpenSteer/PlugIn.h>
#include "common.h"

namespace ossup
{

using namespace OpenSteer;

// ----------------------------------------------------------------------------
// This PlugIn uses two vehicle types: MpWanderer and MpPursuer.  They have
// a common base class, MpBase, which is a specialization of SimpleVehicle.

template<typename Entity>
class MpBase: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a mp base: an STL vector of MpBase pointers
	typedef typename std::vector<MpBase<Entity>*> groupType;

	// constructor
	MpBase()
	{
		reset();
	}

	// reset state
	void reset(void)
	{
		SimpleVehicle::reset(); // reset the vehicle
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();
///		this->setSpeed(0);            // speed along Forward direction.
///		this->setMaxForce(5.0);       // steering force is clipped to this magnitude
///		this->setMaxSpeed(3.0);       // velocity is clipped to this magnitude
#ifdef OS_DEBUG
		this->clearTrailHistory();    // prevent long streaks due to teleportation
		this->gaudyPursuitAnnotation = true; // select use of 9-color annotation
#endif
	}

#ifdef OS_DEBUG
	// draw into the scene
	void draw(void)
	{
		drawBasic2dCircularVehicle(*this, bodyColor);
		this->drawTrail();
	}

	// for draw method
	Color bodyColor;
#endif
};

template<typename Entity>
class MpWanderer: public MpBase<Entity>
{
public:

	// constructor
	MpWanderer()
	{
		reset();
	}

	// reset state
	void reset(void)
	{
		MpBase<Entity>::reset();
#ifdef OS_DEBUG
		this->bodyColor.set(0.4f, 0.6f, 0.4f); // greenish
#endif
	}

	// one simulation step
	void update(const float currentTime, const float elapsedTime)
	{
		const Vec3 wander2d = this->steerForWander(elapsedTime).setYtoZero();
		const Vec3 steer = this->forward() + (wander2d * 3);
		this->applySteeringForce(steer, elapsedTime);

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// for annotation
		this->recordTrailVertex(currentTime, this->position());
#endif
	}

};

//MpWanderer externally updated.
template<typename Entity>
class ExternalMpWanderer: public MpWanderer<Entity>
{
public:
	void update(const float currentTime, const float elapsedTime)
	{
		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// for annotation
		this->recordTrailVertex(currentTime, this->position());
#endif
	}
};

template<typename Entity>
class MpPursuer: public MpBase<Entity>
{
public:

	// constructor
///	MpPursuer(MpWanderer* w)
	MpPursuer()
	{
		wanderer = NULL;
		reset();
	}

	// reset state
	void reset(void)
	{
		MpBase<Entity>::reset();
#ifdef OS_DEBUG
		this->bodyColor.set(0.6f, 0.4f, 0.4f); // redish
#endif
///		randomizeStartingPositionAndHeading();
	}

	// one simulation step
	void update(const float currentTime, const float elapsedTime)
	{
		if (! wanderer)
		{
			return;
		}
		// when pursuer touches quarry ("wanderer"), reset its position
		const float d = Vec3::distance(this->position(), wanderer->position());
		const float r = this->radius() + wanderer->radius();
		if (d < r)
		{
			reset();
		}

		const float maxTime = 20; // xxx hard-to-justify value
		//XXX put "steering force for pursuit" constrained to global XZ "ground" plane
		this->applySteeringForce(this->steerForPursuit(*wanderer, maxTime), elapsedTime);

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// for annotation
		this->recordTrailVertex(currentTime, this->position());
#endif
	}

	// randomize heading only
	void randomizeStartingPositionAndHeading(void)
	{
///		// randomize position on a ring between inner and outer radii
///		// centered around the home base
///		const float inner = 20;
///		const float outer = 30;
///		const float radius = frandom2(inner, outer);
///		const Vec3 randomOnRing = RandomUnitVectorOnXZPlane() * radius;
///		this->setPosition(wanderer->position() + randomOnRing);

		// randomize 2D heading
		this->randomizeHeadingOnXZPlane();
	}

	MpWanderer<Entity>* wanderer;
};

//MpPursuer externally updated.
template<typename Entity>
class ExternalMpPursuer: public MpPursuer<Entity>
{
public:
	void update(const float currentTime, const float elapsedTime)
	{
		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// for annotation
		this->recordTrailVertex(currentTime, this->position());
#endif
	}
};

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo

template<typename Entity>
class MpPlugIn: public PlugIn
{
public:

	const char* name(void)
	{
		return "Multiple Pursuit";
	}

	float selectionOrderSortKey(void)
	{
		return 0.04f;
	}

	virtual ~MpPlugIn()
	{
	} // be more "nice" to avoid a compiler warning

	void open(void)
	{
///		// create the wanderer, saving a pointer to it
///		wanderer = new MpWanderer;
		wanderer = NULL;
///		allMP.push_back(wanderer);
///		// create the specified number of pursuers, save pointers to them
///		const int pursuerCount = 30;
///		for (int i = 0; i < pursuerCount; i++)
///			allMP.push_back(new MpPursuer(wanderer));
///		pBegin = allMP.begin() + 1;  // iterator pointing to first pursuer
///		pEnd = allMP.end();          // iterator pointing to last pursuer

	}

	void update(const float currentTime, const float elapsedTime)
	{
		// update each vehicles
		iterator iter;
		for (iter = allMP.begin(); iter != allMP.end(); ++iter)
		{
			(*iter)->update(currentTime, elapsedTime);
		}
	}

	void redraw(const float currentTime, const float elapsedTime)
	{
#ifdef OS_DEBUG
		// draw each vehicles
		iterator iter;
		for (iter = allMP.begin(); iter != allMP.end(); ++iter)
		{
			(*iter)->draw();
		}
#endif
	}

	void close(void)
	{
		// delete wanderer, all pursuers, and clear list
///		delete (wanderer);
///		for (iterator i = pBegin; i != pEnd; i++)
///			delete ((MpPursuer*) *i);
///		allMP.clear();
	}

	void reset(void)
	{
		// reset each vehicles
		iterator iter;
		for (iter = allMP.begin(); iter != allMP.end(); ++iter)
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
		//check if this is a MpWanderer
		MpWanderer<Entity>* wandererTmp =
				dynamic_cast<MpWanderer<Entity>*>(vehicle);
		if (wandererTmp)
		{
			// set the plugin's wanderer: the last added one
			wanderer = wandererTmp;
			//update each pursuer's wanderer
			setAllPursuersWanderer();
			//that's all
			return true;
		}
		//or if this is a MpPursuer
		MpPursuer<Entity>* pursuerTmp =
			dynamic_cast<MpPursuer<Entity>*>(vehicle);
		if (pursuerTmp)
		{
			//if not ExternalMpPursuer then randomize
			if (! dynamic_cast<ExternalMpPursuer<Entity>*>(pursuerTmp))
			{
				// randomize only 2D heading
				pursuerTmp->randomizeStartingPositionAndHeading();
			}
			// set the pursuer's wanderer
			pursuerTmp->wanderer = wanderer;
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
		if (! PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle))
		{
			return false;
		}
		//check if this is the current wanderer
		if (vehicle == wanderer)
		{
			wanderer = NULL;
			// find a new wanderer (if any): the first found or NULL
			iterator iter;
			for (iter = allMP.begin(); iter != allMP.end(); ++iter)
			{
				wanderer = dynamic_cast<MpWanderer<Entity>*>(*iter);
				if (wanderer)
				{
					break;
				}
			}
			//update each pursuer's wanderer
			setAllPursuersWanderer();
		}
		//
		return true;
	}

	void setAllPursuersWanderer()
	{
		iterator iter;
		for (iter = allMP.begin(); iter != allMP.end(); ++iter)
		{
			MpPursuer<Entity>* pursuerTmp =
					dynamic_cast<MpPursuer<Entity>*>(*iter);
			if (pursuerTmp)
			{
				// update the pursuer's wanderer
				pursuerTmp->wanderer = wanderer;
			}
		}
	}

	const AVGroup& allVehicles(void)
	{
		return (const AVGroup&) allMP;
	}

	// a group (STL vector) of all vehicles
	typename MpBase<Entity>::groupType allMP;
	typedef typename MpBase<Entity>::groupType::const_iterator iterator;

///	iterator pBegin, pEnd;

	MpWanderer<Entity>* wanderer;
};

//MpPlugIn gMpPlugIn;

// ----------------------------------------------------------------------------

} // ossup namespace

#endif /* PLUGIN_MULTIPLEPURSUIT_H_ */
