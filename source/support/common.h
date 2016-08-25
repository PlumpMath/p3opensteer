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

///Macros
#ifdef OS_DEBUG
#	define PRINT_DEBUG(msg) std::cout << msg << std::endl
#	define PRINT_ERR_DEBUG(msg) std::cerr << msg << std::endl
#	define ASSERT_TRUE(cond) \
		if (!(cond)) { \
		  std::cerr << "assertion error : (" << #cond << ") at " \
		  << __LINE__ << ", " \
		  << __FILE__ << std::endl; \
		}
#else
#	define PRINT_DEBUG(msg)
#	define PRINT_ERR_DEBUG(msg)
#	define ASSERT_TRUE(cond)
#endif

#define RETURN_ON_COND(_flag_,_return_)\
	if (_flag_)\
	{\
		return _return_;\
	}

extern ossup::DrawMeshDrawer *gDrawer3d, *gDrawer2d;

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

typedef std::vector<OpenSteer::SegmentedPathway*> PathwayGroup;

/**
 * \brief Vehicle settings.
 */
struct VehicleSettings
{
	VehicleSettings() :
			m_mass(1.0), m_radius(1.0), m_speed(0.0), m_maxForce(1.0),
			m_maxSpeed(1.0), m_forward(OpenSteer::Vec3::forward),
			m_side(OpenSteer::Vec3::side), m_up(OpenSteer::Vec3::up),
			m_position(OpenSteer::Vec3::zero), m_start(OpenSteer::Vec3::zero),
			m_pathPredTime(3.0), m_obstacleMinTimeColl(6.0),
			m_neighborMinTimeColl(3.0), m_neighborMinSepDist(1.0),
			m_separationMaxDist(5.0), m_separationCosMaxAngle(-0.707),
			m_alignmentMaxDist(7.5), m_alignmentCosMaxAngle(0.7),
			m_cohesionMaxDist(9.0), m_cohesionCosMaxAngle(-0.15),
			m_pursuitMaxPredTime(20.0), m_evasionMaxPredTime(20.0),
			m_targetSpeed(1.0)
	{
	}
	bool operator == (const VehicleSettings& other)
	{
		return (m_mass == other.m_mass) &&
				(m_radius == other.m_radius) &&
				(m_speed == other.m_speed) &&
				(m_maxForce == other.m_maxForce) &&
				(m_maxSpeed == other.m_maxSpeed) &&
				(m_forward == other.m_forward) &&
				(m_side == other.m_side) &&
				(m_up == other.m_up) &&
				(m_position == other.m_position) &&
				(m_start == other.m_start) &&
				(m_pathPredTime == other.m_pathPredTime) &&
				(m_obstacleMinTimeColl == other.m_obstacleMinTimeColl) &&
				(m_neighborMinTimeColl == other.m_neighborMinTimeColl) &&
				(m_neighborMinSepDist == other.m_neighborMinSepDist) &&
				(m_separationMaxDist == other.m_separationMaxDist) &&
				(m_separationCosMaxAngle == other.m_separationCosMaxAngle) &&
				(m_alignmentMaxDist == other.m_alignmentMaxDist) &&
				(m_alignmentCosMaxAngle == other.m_alignmentCosMaxAngle) &&
				(m_cohesionMaxDist == other.m_cohesionMaxDist) &&
				(m_cohesionCosMaxAngle == other.m_cohesionCosMaxAngle) &&
				(m_pursuitMaxPredTime == other.m_pursuitMaxPredTime) &&
				(m_evasionMaxPredTime == other.m_evasionMaxPredTime) &&
				(m_targetSpeed == other.m_targetSpeed);
	}
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
	// the vehicle start position.
	OpenSteer::Vec3 m_start;
	// steering parameters
	float m_pathPredTime;//steerToFollowPath, steerToStayOnPath
	float m_obstacleMinTimeColl;//steerToAvoidObstacle, steerToAvoidObstacles
	float m_neighborMinTimeColl;//steerToAvoidNeighbors
	float m_neighborMinSepDist;//steerToAvoidCloseNeighbors
	float m_separationMaxDist;//steerForSeparation
	float m_separationCosMaxAngle;//steerForSeparation
	float m_alignmentMaxDist;//steerForAlignment
	float m_alignmentCosMaxAngle;//steerForAlignment
	float m_cohesionMaxDist;//steerForCohesion
	float m_cohesionCosMaxAngle;//steerForCohesion
	float m_pursuitMaxPredTime;//steerForPursuit
	float m_evasionMaxPredTime;//steerForEvasion
	float m_targetSpeed;//steerForTargetSpeed
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
	virtual ~VehicleAddOnMixin(){}

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
		m_settings.m_mass = Super::mass();
		m_settings.m_radius = Super::radius();
		m_settings.m_speed = Super::speed(); // speed along Forward direction.
		m_settings.m_maxForce = Super::maxForce(); // steering force is clipped to this magnitude
		m_settings.m_maxSpeed = Super::maxSpeed(); // velocity is clipped to this magnitude
		m_settings.m_forward = Super::forward();
		m_settings.m_side = Super::side();
		m_settings.m_up = Super::up();
		m_settings.m_position = Super::position();
		return m_settings;
	}

	void setSettings(const VehicleSettings& settings)
	{
		m_settings = settings;
		//set vehicle settings effectively
		reset();
	}

	OpenSteer::Vec3 getStart() const
	{
		return m_settings.m_start;
	}
	void setStart(const OpenSteer::Vec3& start)
	{
		m_settings.m_start = start;
	}

	float getPathPredTime() const
	{
		return m_settings.m_pathPredTime;
	}
	void setPathPredTime(float value)
	{
		m_settings.m_pathPredTime = value;
	}

	float getObstacleMinTimeColl() const
	{
		return m_settings.m_obstacleMinTimeColl;
	}
	void setObstacleMinTimeColl(float value)
	{
		m_settings.m_obstacleMinTimeColl = value;
	}

	float getNeighborMinTimeColl() const
	{
		return m_settings.m_neighborMinTimeColl;
	}
	void setNeighborMinTimeColl(float value)
	{
		m_settings.m_neighborMinTimeColl = value;
	}

	float getNeighborMinSepDist() const
	{
		return m_settings.m_neighborMinSepDist;
	}
	void setNeighborMinSepDist(float value)
	{
		m_settings.m_neighborMinSepDist = value;
	}

	float getSeparationMaxDist() const
	{
		return m_settings.m_separationMaxDist;
	}
	void setSeparationMaxDist(float value)
	{
		m_settings.m_separationMaxDist = value;
	}

	float getSeparationCosMaxAngle() const
	{
		return m_settings.m_separationCosMaxAngle;
	}
	void setSeparationCosMaxAngle(float value)
	{
		m_settings.m_separationCosMaxAngle = value;
	}

	float getAlignmentMaxDist() const
	{
		return m_settings.m_alignmentMaxDist;
	}
	void setAlignmentMaxDist(float value)
	{
		m_settings.m_alignmentMaxDist = value;
	}

	float getAlignmentCosMaxAngle() const
	{
		return m_settings.m_alignmentCosMaxAngle;
	}
	void setAlignmentCosMaxAngle(float value)
	{
		m_settings.m_alignmentCosMaxAngle = value;
	}

	float getCohesionMaxDist() const
	{
		return m_settings.m_cohesionMaxDist;
	}
	void setCohesionMaxDist(float value)
	{
		m_settings.m_cohesionMaxDist = value;
	}

	float getCohesionCosMaxAngle() const
	{
		return m_settings.m_cohesionCosMaxAngle;
	}
	void setCohesionCosMaxAngle(float value)
	{
		m_settings.m_cohesionCosMaxAngle = value;
	}

	float getPursuitMaxPredTime() const
	{
		return m_settings.m_pursuitMaxPredTime;
	}
	void setPursuitMaxPredTime(float value)
	{
		m_settings.m_pursuitMaxPredTime = value;
	}

	float getEvasionMaxPredTime() const
	{
		return m_settings.m_evasionMaxPredTime;
	}
	void setEvasionMaxPredTime(float value)
	{
		m_settings.m_evasionMaxPredTime = value;
	}

	float getTargetSpeed() const
	{
		return m_settings.m_targetSpeed;
	}
	void setTargetSpeed(float value)
	{
		m_settings.m_targetSpeed = value;
	}

	/// It is called by setSettings() to perform custom tasks
	/// which usually depend on base settings
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
	}

#ifdef OS_DEBUG
	virtual void draw()
	{
	}
#endif

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
			obstacles(NULL), localObstacles(NULL), selectedVehicle(NULL)
	{
		m_pathway.resize(1);
		m_pathway[0] = NULL;
	}

	virtual ~PlugInAddOnMixin()
	{
		delete m_pathway[0];
		m_pathway.clear();
	}

	///Note: addVehicle() derived function must not change any vehicle's
	///physical characteristics as defined by ossup::VehicleSettings
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
		delete m_pathway[0];
		m_pathway[0] = NULL;
		//create a new pathway
		if (numOfPoints < 2)
		{
			m_pathway[0] = NULL;
			return;
		}
		//check if single radius
		if (singleRadius)
		{
			m_pathway[0] = new OpenSteer::PolylineSegmentedPathwaySingleRadius(
					numOfPoints, points, radii[0], closedCycle);
		}
		else
		{
			m_pathway[0] = new OpenSteer::PolylineSegmentedPathwaySegmentRadii(
					numOfPoints, points, radii, closedCycle);
		}
	}

	//will compute point, radius
	void getPathwayEndPointData(int idx0, int idx1,
			OpenSteer::Vec3& pathEndpoint0, OpenSteer::Vec3& pathEndpoint1,
			float& radiusEndpoint0, float&radiusEndpoint1)
	{
		int maxPointIdx = (int) m_pathway[0]->pointCount() - 1;
		//check bounds
		if ((idx0 < 0) || (idx0 > maxPointIdx) || (idx1 < 0)
				|| (idx1 > maxPointIdx))
		{
			return;
		}
		pathEndpoint0 = m_pathway[0]->point(idx0);
		pathEndpoint1 = m_pathway[0]->point(idx1);
		OpenSteer::PolylineSegmentedPathwaySegmentRadii* pathRadii =
				dynamic_cast<OpenSteer::PolylineSegmentedPathwaySegmentRadii*>(m_pathway[0]);
		if (pathRadii)
		{
			//depending if path is closed cycle:
			//segment count == point count || point count - 1
			int maxSegmentIdx = (int) pathRadii->segmentCount() - 1;
			radiusEndpoint0 = pathRadii->segmentRadius(
					min(idx0, maxSegmentIdx));
			radiusEndpoint1 = pathRadii->segmentRadius(
					min(idx1, maxSegmentIdx));
		}
		else
		{
			//single radius
			float radius =
					static_cast<OpenSteer::PolylineSegmentedPathwaySingleRadius*>(m_pathway[0])->radius();
			radiusEndpoint0 = radius;
			radiusEndpoint1 = radius;
		}
	}

#ifdef OS_DEBUG
	virtual void drawPath(void)
	{
		typedef OpenSteer::SegmentedPathway::size_type size_type;

		// draw a line along each segment of path
		gDrawer3d->setTwoSided(true);
		for (size_type i = 1; i < (*m_pathway[0]).pointCount(); ++i)
		{
			drawLine((*m_pathway[0]).point(i), (*m_pathway[0]).point(i - 1),
					OpenSteer::gRed);
		}
		gDrawer3d->setTwoSided(false);
	}

	virtual void drawObstacles(void)
	{
		gDrawer3d->setTwoSided(true);
		// draw obstacles
		OpenSteer::ObstacleIterator iterObstacle;
		for (iterObstacle = localObstacles->begin();
				iterObstacle != localObstacles->end(); ++iterObstacle)
		{
			(*iterObstacle)->draw(false, OpenSteer::Color(0, 0, 0),
					OpenSteer::Vec3(0, 0, 0));
		}
		gDrawer3d->setTwoSided(false);
	}
#endif

	///A reference to global obstacles handled by all plugins.
	OpenSteer::ObstacleGroup* obstacles;
	///A reference to local obstacles handled by this plugins.
	OpenSteer::ObstacleGroup* localObstacles;

protected:
	///The selected vehicle (for debug draw only).
	OpenSteer::AbstractVehicle* selectedVehicle;
	///The pathway(s) handled by this plugin.
	PathwayGroup m_pathway;
};

//common typedef
typedef PlugInAddOnMixin<OpenSteer::PlugIn> PlugIn;

} // namespace ossup

#endif /* OPENSTEERLOCALCOMMON_H_ */
