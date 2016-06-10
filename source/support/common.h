/**
 * \file common.h
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OPENSTEERLOCALCOMMON_H_
#define OPENSTEERLOCALCOMMON_H_

#include <algorithm>
#include <nodePath.h>
#include <OpenSteer/Vec3.h>
#include <OpenSteer/Color.h>
#include <OpenSteer/PlugIn.h>
#include <OpenSteer/PolylineSegmentedPathwaySegmentRadii.h>
#include <OpenSteer/PolylineSegmentedPathwaySingleRadius.h>
#include "SimpleVehicle.h"
#include "DrawMeshDrawer.h"

extern ossup::DrawMeshDrawer *gDrawer3d, *gDrawer2d;
extern ReMutex gOpenSteerDebugMutex;

namespace ossup
{

//LVecBase3f-OpenSteer::Vec3 conversion functions
inline OpenSteer::Vec3 LVecBase3fToOpenSteerVec3(const LVecBase3f& v)
{
	return OpenSteer::Vec3(v.get_x(), v.get_z(), -v.get_y());
}
inline LVecBase3f OpenSteerVec3ToLVecBase3f(const OpenSteer::Vec3& v)
{
	return LVecBase3f(v.x, -v.z, v.y);
}
//LVecBase4f-OpenSteer::Color conversion functions
inline OpenSteer::Color LVecBase4fToOpenSteerColor(const LVecBase4f& c)
{
	return OpenSteer::Color(c.get_x(), c.get_y(), c.get_z(), c.get_w());
}
inline LVecBase4f OpenSteerColorToLVecBase4f(const OpenSteer::Color& c)
{
	return LVecBase4f(c.r(), c.g(), c.b(), c.a());
}

/**
 * \brief Vehicle settings.
 */
struct VehicleSettings
{
	// mass
	float m_mass;
	// size of bounding sphere, for obstacle avoidance, etc.
	float m_radius;
	// speed of vehicle
	float m_speed;
	// the maximum steering force this vehicle can apply
	float m_maxForce;
	// the maximum speed this vehicle is allowed to move
	float m_maxSpeed;
	// forward-pointing unit basis vector
	OpenSteer::Vec3 m_forward;
	// side-pointing unit basis vector
	OpenSteer::Vec3 m_side;
	// upward-pointing unit basis vector
	OpenSteer::Vec3 m_up;
	// origin of local space
	OpenSteer::Vec3 m_position;
};

template<typename Super, typename Entity>
class VehicleAddOnMixin: public Super
{
public:

	VehicleAddOnMixin() :
			m_entity(NULL), m_entityUpdateMethod(NULL), m_entityPathFollowingMethod(
			NULL), m_entityAvoidObstacleMethod(NULL), m_entityAvoidCloseNeighborMethod(
			NULL), m_entityAvoidNeighborMethod(NULL)
	{
	}

	typedef Entity* ENTITY;
	typedef void (Entity::*ENTITYUPDATEMETHOD)(const float, const float);
	typedef void (Entity::*ENTITYPATHFOLLOWINGMETHOD)(const OpenSteer::Vec3&,
			const OpenSteer::Vec3&, const OpenSteer::Vec3&, const float);
	typedef void (Entity::*ENTITYAVOIDOBSTACLEMETHOD)(const float);
	typedef void (Entity::*ENTITYAVOIDCLOSENEIGHBORMETHOD)(
			const OpenSteer::AbstractVehicle&, const float);
	typedef void (Entity::*ENTITYAVOIDNEIGHBORMETHOD)(
			const OpenSteer::AbstractVehicle&, const float,
			const OpenSteer::Vec3&, const OpenSteer::Vec3&);

	void setEntity(ENTITY entity)
	{
		m_entity = entity;
	}

	void setEntityUpdateMethod(ENTITYUPDATEMETHOD entityUpdateMethod)
	{
		m_entityUpdateMethod = entityUpdateMethod;
	}

	void entityUpdate(const float currentTime, const float elapsedTime)
	{
		(m_entity->*m_entityUpdateMethod)(currentTime, elapsedTime);
	}

    virtual void annotatePathFollowing (const OpenSteer::Vec3& future,
                                        const OpenSteer::Vec3& onPath,
                                        const OpenSteer::Vec3& target,
                                        const float outside)
    {
		///call the m_entityPathFollowingMethod
		if (m_entityPathFollowingMethod)
		{
			(m_entity->*m_entityPathFollowingMethod)(future, onPath, target,
					outside);
		}
    }
    void setEntityPathFollowingMethod(
			ENTITYPATHFOLLOWINGMETHOD entityPathFollowingMethod)
	{
		m_entityPathFollowingMethod = entityPathFollowingMethod;
	}

    virtual void annotateAvoidObstacle (const float minDistanceToCollision)
    {
		///call the m_entityAvoidObstacleMethod
		if (m_entityAvoidObstacleMethod)
		{
			(m_entity->*m_entityAvoidObstacleMethod)(minDistanceToCollision);
		}
    }
	void setEntityAvoidObstacleMethod(
			ENTITYAVOIDOBSTACLEMETHOD entityAvoidObstacleMethod)
	{
		m_entityAvoidObstacleMethod = entityAvoidObstacleMethod;
	}

    virtual void annotateAvoidCloseNeighbor (const OpenSteer::AbstractVehicle& other,
                                             const float additionalDistance)
    {
		///call the m_entityAvoidCloseNeighborMethod
		if (m_entityAvoidCloseNeighborMethod)
		{
			(m_entity->*m_entityAvoidCloseNeighborMethod)(other,
					additionalDistance);
		}
    }
	void setEntityAvoidCloseNeighborMethod(
			ENTITYAVOIDCLOSENEIGHBORMETHOD entityAvoidCloseNeighborMethod)
	{
		m_entityAvoidCloseNeighborMethod = entityAvoidCloseNeighborMethod;
	}

    virtual void annotateAvoidNeighbor (const OpenSteer::AbstractVehicle& threat,
                                        const float steer,
                                        const OpenSteer::Vec3& ourFuture,
                                        const OpenSteer::Vec3& threatFuture)
    {
		///call the m_entityAvoidNeighborMethod
		if (m_entityAvoidNeighborMethod)
		{
			(m_entity->*m_entityAvoidNeighborMethod)(threat, steer, ourFuture,
					threatFuture);
		}
    }
    void setEntityAvoidNeighborMethod(
			ENTITYAVOIDNEIGHBORMETHOD entityAvoidNeighborMethod)
	{
		m_entityAvoidNeighborMethod = entityAvoidNeighborMethod;
	}

    VehicleSettings& getSettings()
	{
		return m_settings;
	}

	void setSettings(const VehicleSettings& settings)
	{
		m_settings = settings;
		//set vehicle settings effectivossup
		reset();
	}

	virtual void reset()
	{
		Super::setMass(m_settings.m_mass);
		Super::setRadius(m_settings.m_radius);
		Super::setSpeed(m_settings.m_speed); // speed along Forward direction.
		Super::setMaxForce(m_settings.m_maxForce); // steering force is clipped to this magnitude
		Super::setMaxSpeed(m_settings.m_maxSpeed); // velocity is clipped to this magnitude
		Super::setForward(m_settings.m_forward);
		Super::setSide(Super::localRotateForwardToSide(m_settings.m_forward));
		Super::setUp(m_settings.m_up);
		Super::setPosition(m_settings.m_position);
		m_start = Super::position();
	}

#ifdef OS_DEBUG
	virtual void draw()
	{
	}
#endif

	OpenSteer::Vec3 getStart()
	{
		return m_start;
	}

protected:
	///The entity updated by the vehicle.
	ENTITY m_entity;
	///The entity methods.
	///@{
	ENTITYUPDATEMETHOD m_entityUpdateMethod;
	ENTITYPATHFOLLOWINGMETHOD m_entityPathFollowingMethod;
	ENTITYAVOIDOBSTACLEMETHOD m_entityAvoidObstacleMethod;
	ENTITYAVOIDCLOSENEIGHBORMETHOD m_entityAvoidCloseNeighborMethod;
	ENTITYAVOIDNEIGHBORMETHOD m_entityAvoidNeighborMethod;
	///@}
	///The vehicle settings.
	VehicleSettings m_settings;
	///The vehicle start position.
	OpenSteer::Vec3 m_start;
};

//Obstacles: redefinition of draw
class SphereObstacle: public OpenSteer::SphereObstacle
{
public:
	SphereObstacle(float r, OpenSteer::Vec3 c) :
			OpenSteer::SphereObstacle(r, c)
	{
	}
#ifdef OS_DEBUG
	virtual void draw(const bool filled, const OpenSteer::Color& color,
			const OpenSteer::Vec3& viewpoint) const
	{
		OpenSteer::Color yellow(1, 1, 0, 1);
		//y axis
		draw3dCircleOrDisk(radius, center, OpenSteer::Vec3(0, 1, 0), yellow, 20,
				false);
		//x axis
		draw3dCircleOrDisk(radius, center, OpenSteer::Vec3(1, 0, 0), yellow, 20,
				false);
	}
#endif
};

class BoxObstacle: public OpenSteer::BoxObstacle
{
public:
	BoxObstacle(float w, float h, float d) :
			OpenSteer::BoxObstacle(w, h, d)
	{
	}
#ifdef OS_DEBUG
	virtual void draw(const bool filled, const OpenSteer::Color& color,
			const OpenSteer::Vec3& viewpoint) const
	{
		OpenSteer::Color brown(0.647, 0.164, 0.164, 1);
		float w = width * 0.5f;
		float h = height * 0.5f;
		float d = depth * 0.5f;

		OpenSteer::Vec3 v1 = globalizePosition(OpenSteer::Vec3(w, h, d));
		OpenSteer::Vec3 v2 = globalizePosition(OpenSteer::Vec3(-w, h, d));
		OpenSteer::Vec3 v3 = globalizePosition(OpenSteer::Vec3(-w, -h, d));
		OpenSteer::Vec3 v4 = globalizePosition(OpenSteer::Vec3(w, -h, d));
		OpenSteer::Vec3 v5 = globalizePosition(OpenSteer::Vec3(w, h, -d));
		OpenSteer::Vec3 v6 = globalizePosition(OpenSteer::Vec3(-w, h, -d));
		OpenSteer::Vec3 v7 = globalizePosition(OpenSteer::Vec3(-w, -h, -d));
		OpenSteer::Vec3 v8 = globalizePosition(OpenSteer::Vec3(w, -h, -d));

		//sides
		drawLine(v1, v2, brown);
		drawLine(v2, v3, brown);
		drawLine(v3, v4, brown);
		drawLine(v4, v1, brown);
		drawLine(v5, v6, brown);
		drawLine(v6, v7, brown);
		drawLine(v7, v8, brown);
		drawLine(v8, v5, brown);
		drawLine(v1, v5, brown);
		drawLine(v2, v6, brown);
		drawLine(v3, v7, brown);
		drawLine(v4, v8, brown);
	}
#endif
};

class PlaneObstacle: public OpenSteer::PlaneObstacle
{
public:
	PlaneObstacle(const OpenSteer::Vec3& s, const OpenSteer::Vec3& u,
			const OpenSteer::Vec3& f, const OpenSteer::Vec3& p) :
			OpenSteer::PlaneObstacle(s, u, f, p)
	{
	}
#ifdef OS_DEBUG
	virtual void draw(const bool filled, const OpenSteer::Color& color,
			const OpenSteer::Vec3& viewpoint) const
	{
		OpenSteer::Color skyblue(0, 1, 1, 1);
		drawLine(position() - (forward() * 20), position() + (forward() * 20),
				skyblue);
		drawLine(position() - (side() * 20), position() + (side() * 20),
				skyblue);
	}
#endif
};

class RectangleObstacle: public OpenSteer::RectangleObstacle
{
public:
	RectangleObstacle(float w, float h, const OpenSteer::Vec3& s,
			const OpenSteer::Vec3& u, const OpenSteer::Vec3& f,
			const OpenSteer::Vec3& p, seenFromState sf) :
			OpenSteer::RectangleObstacle(w, h, s, u, f, p, sf)
	{
	}
#ifdef OS_DEBUG
	virtual void draw(const bool filled, const OpenSteer::Color& color,
			const OpenSteer::Vec3& viewpoint) const
	{
		OpenSteer::Color purple(1, 0, 1, 1);
		float w = width * 0.5f;
		float h = height * 0.5f;

		OpenSteer::Vec3 v1 = globalizePosition(OpenSteer::Vec3(w, h, 0));
		OpenSteer::Vec3 v2 = globalizePosition(OpenSteer::Vec3(-w, h, 0));
		OpenSteer::Vec3 v3 = globalizePosition(OpenSteer::Vec3(-w, -h, 0));
		OpenSteer::Vec3 v4 = globalizePosition(OpenSteer::Vec3(w, -h, 0));

		//sides
		drawLine(v1, v2, purple);
		drawLine(v2, v3, purple);
		drawLine(v3, v4, purple);
		drawLine(v4, v1, purple);
	}
#endif
};

template<typename Super>
class PlugInAddOnMixin: public Super
{
public:

	PlugInAddOnMixin() :
			obstacles(NULL), localObstacles(NULL), selectedVehicle(NULL), m_pathway(
			NULL)
	{
	}

	virtual ~PlugInAddOnMixin()
	{
		delete m_pathway;
	}

	virtual bool addVehicle(OpenSteer::AbstractVehicle* vehicle)
	{
		bool result = false;
		OpenSteer::AVGroup::iterator iter;
		//check if vehicle has not been already added
		for (iter = const_cast<OpenSteer::AVGroup&>(this->allVehicles()).begin();
				iter
						!= const_cast<OpenSteer::AVGroup&>(this->allVehicles()).end();
				++iter)
		{
			if (*iter == vehicle)
			{
				break;
			}
		}
		if (iter == this->allVehicles().end())
		{
			//vehicle needs to be added
			(const_cast<OpenSteer::AVGroup&>(this->allVehicles())).push_back(
					vehicle);
			//this is the new selected vehicle
			selectedVehicle = vehicle;
			//set result
			result = true;
		}
		//
		return result;
	}

	virtual bool removeVehicle(OpenSteer::AbstractVehicle* vehicle)
	{
		bool result = false;
		OpenSteer::AVGroup::iterator iter;
		//check if vehicle has been already removed or not
		for (iter = const_cast<OpenSteer::AVGroup&>(this->allVehicles()).begin();
				iter
						!= const_cast<OpenSteer::AVGroup&>(this->allVehicles()).end();
				++iter)
		{
			if (*iter == vehicle)
			{
				break;
			}
		}
		if (iter != this->allVehicles().end())
		{
			//vehicle needs to be removed
			(const_cast<OpenSteer::AVGroup&>(this->allVehicles())).erase(iter);
			//if this is the selected vehicle set to NULL
			if (selectedVehicle == vehicle)
			{
				selectedVehicle = NULL;
			}
			//set result
			result = true;
		}
		//
		return result;
	}

	void setSelectedVehicle(OpenSteer::AbstractVehicle* _selectedVehicle)
	{
		selectedVehicle = _selectedVehicle;
	}

	OpenSteer::Pathway* getPathway()
	{
		return m_pathway;
	}

	void setPathway(OpenSteer::SegmentedPathway::size_type numOfPoints,
			OpenSteer::Vec3 const points[], bool singleRadius,
			float const radii[], bool closedCycle)
	{
		//delete old pathway
		delete m_pathway;
		m_pathway = NULL;
		//create a new pathway
		if (numOfPoints < 2)
		{
			m_pathway = NULL;
			return;
		}
		//check if single radius
		if (singleRadius)
		{
			m_pathway = new OpenSteer::PolylineSegmentedPathwaySingleRadius(
					numOfPoints, points, radii[0], closedCycle);
		}
		else
		{
			m_pathway = new OpenSteer::PolylineSegmentedPathwaySegmentRadii(
					numOfPoints, points, radii, closedCycle);
		}
		pathEndpoint0 = points[0];
		pathEndpoint1 = points[numOfPoints - 1];
	}

	///A reference to global obstacles handled by all plugins.
	OpenSteer::ObstacleGroup* obstacles;
	///A reference to local obstacles handled by this plugins.
	OpenSteer::ObstacleGroup* localObstacles;

protected:
	///The selected vehicle (for debug draw only).
	OpenSteer::AbstractVehicle* selectedVehicle;
	///The pathway handled by this plugin.
	OpenSteer::Pathway* m_pathway;
	///The pathway endpoints.
	OpenSteer::Vec3 pathEndpoint0, pathEndpoint1;
};

//common typedef
typedef PlugInAddOnMixin<OpenSteer::PlugIn> PlugIn;

} // namespace ossup

#endif /* OPENSTEERLOCALCOMMON_H_ */
