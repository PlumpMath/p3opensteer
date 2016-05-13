/**
 * \file osSteerVehicle.h
 *
 * \date 2016-05-13
 * \author consultit
 */
#ifndef OSSTEERVEHICLE_H_
#define OSSTEERVEHICLE_H_

#include "ObjectModel/PandaNode.h"
#include "ObjectModel/Object.h"
#include "Support/OpenSteerLocal/common.h"
#include <bulletWorld.h>
#include <bulletClosestHitRayResult.h>
#include <throw_event.h>

class SteerVehicleTemplate;
class SteerPlugIn;

///OSSteerVehicle movement type.
enum SteerVehicleMovType
{
	OPENSTEER,
	OPENSTEER_KINEMATIC,
	VehicleMovType_NONE
};

/**
 * \brief Component implementing OpenSteer Vehicles.
 *
 * \see http://opensteer.sourceforge.net
 *
 * This component should be associated to a "Scene" component.\n
 * If specified in "thrown_events", this component can throw
 * these events (shown with default names):
 * - on moving (<ObjectType>_SteerVehiclet_Move)
 * - on being steady (<ObjectType>_SteerVehicle_Steady)
 * - when steering is required to follow a path
 * (<ObjectType>_SteerVehicle_PathFollowing)
 * - when steering is required to avoid an obstacle
 * (<ObjectType>_SteerVehicle_AvoidObstacle)
 * - when steering is required to avoid a close neighbor (i.e. when
 * there is a collision) (<ObjectType>_SteerVehicle_AvoidCloseNeighbor)
 * - when steering is required to avoid a neighbor (i.e. when there
 * is a potential collision) (<ObjectType>_SteerVehicle_AvoidNeighbor)
 * Events are thrown continuously at a frequency which is the minimum between
 * the fps and the frequency specified (which defaults to 30 times per seconds).\n
 * The argument of each event is a reference to this component.\n
 * \see annotate* SteerLibraryMixin member functions in SteerLibrary.h
 * for more information.
 *
 * \note debug drawing works correctly only if the owner object's
 * parent is "render".\n
 *
 * > **XML Param(s)**:
 * param | type | default | note
 * ------|------|---------|-----
 * | *thrown_events*			|single| - | specified as "event1@[event_name1]@[frequency1][:...[:eventN@[event_nameN]@[frequencyN]]]" with eventX = move,steady,path_following,avoid_obstacle,avoid_close_neighbor,avoid_neighbor
 * | *type*						|single| *one_turning* | values: one_turning,pedestrian,boid,mp_wanderer,mp_pursuer,player,ball,ctf_seeker,ctf_enemy,low_speed_turn,map_driver
 * | *external_update*			|single| *false* | -
 * | *add_to_plugin*			|single| - | -
 * | *mov_type*					|single| *opensteer* | values: opensteer,kinematic
 * | *up_axis_fixed*			|single| *false* | -
 * | *mass*						|single| 1.0 | -
 * | *radius*					|single| - | -
 * | *speed*					|single| 0.0 | -
 * | *max_force*				|single| 0.1 | -
 * | *max_speed*				|single| 1.0 | -
 * | *ray_mask*					|single| *all_on* | -
 *
 * \note parts inside [] are optional.\n
 */
class OSSteerVehicle: public PandaNode
{
protected:
	friend class SteerVehicleTemplate;
	friend class SteerPlugIn;

	OSSteerVehicle(SMARTPTR(SteerVehicleTemplate)tmpl);
	virtual void reset();
	virtual bool initialize();
	virtual void onAddToObjectSetup();
	virtual void onRemoveFromObjectCleanup();
	virtual void onAddToSceneSetup();
	virtual void onRemoveFromSceneCleanup();

public:
	virtual ~OSSteerVehicle();

	/**
	 * \name Getters/setters of OSSteerVehicle default settings.
	 */
	///@{
	void setSettings(const VehicleSettings& settings);
	VehicleSettings getSettings();
	///@}

	/**
	 * \name AbstractVehicle reference getter & conversion function.
	 */
	///@{
	OpenSteer::AbstractVehicle& getAbstractVehicle();
	operator OpenSteer::AbstractVehicle&();
	///@}

	/**
	 * \name Get the SteerPlugIn owner object.
	 *
	 * \return The SteerPlugIn object.
	 */
	SMARTPTR(SteerPlugIn) getSteerPlugIn() const;

	///OSSteerVehicle thrown events.
	enum EventThrown
	{
		MOVEEVENT,
		STEADYEVENT,
		PATHFOLLOWINGEVENT,
		AVOIDOBSTACLEEVENT,
		AVOIDCLOSENEIGHBOREVENT,
		AVOIDNEIGHBOREVENT
	};

	/**
	 * \brief Enables/disables the OSSteerVehicle event to be thrown.
	 * @param event The OSSteerVehicle event.
	 * @param eventData The OSSteerVehicle event data. ThrowEventData::mEnable
	 * will enable/disable the event.
	 */
	void enableSteerVehicleEvent(EventThrown event, ThrowEventData eventData);

private:
	///Current underlying Vehicle.
	OpenSteer::AbstractVehicle* mVehicle;
	///The SteerPlugIn owner object.
	SMARTPTR(SteerPlugIn) mSteerPlugIn;
	ObjectId mSteerPlugInObjectId;
	///Input radius.
	float mInputRadius;
	///The movement type.
	SteerVehicleMovType mMovType;
	///Flag for up axis fixed (z).
	bool mUpAxisFixed;
	/**
	 * \brief Physics data.
	 */
	///@{
	SMARTPTR(BulletWorld) mBulletWorld;
	float mMaxError;
	LVector3f mDeltaRayDown, mDeltaRayOrig;
	BulletClosestHitRayResult mHitResult;
	BitMask32 mRayMask;
	float mCorrectHeightRigidBody;
	///@}

	///Called by the underlying OpenSteer component update.
	///@{
	void doUpdateSteerVehicle(const float currentTime, const float elapsedTime);
	//Called when component is updated outside of OpenSteer.
	void doExternalUpdateSteerVehicle(const float currentTime, const float elapsedTime);
	bool mExternalUpdate;
	///@}

	/**
	 * \brief SteerLibrary callbacks.
	 */
	///@{
	void doPathFollowing(const OpenSteer::Vec3& future, const OpenSteer::Vec3& onPath,
			const OpenSteer::Vec3& target, const float outside);
	void doAvoidObstacle(const float minDistanceToCollision);
	void doAvoidCloseNeighbor(const OpenSteer::AbstractVehicle& other, const float additionalDistance);
	void doAvoidNeighbor(const OpenSteer::AbstractVehicle& threat, const float steer,
			const OpenSteer::Vec3& ourFuture, const OpenSteer::Vec3& threatFuture);
	///@}

	/**
	 * \name Throwing OSSteerVehicle events.
	 */
	///@{
	bool mPFCallbackCalled, mAOCallbackCalled, mACNCallbackCalled, mANCallbackCalled;
	ThrowEventData mMove, mSteady, mPathFollowing, mAvoidObstacle,
	mAvoidCloseNeighbor, mAvoidNeighbor;
	///Helper.
	void doEnableSteerVehicleEvent(EventThrown event, ThrowEventData eventData);
	void doThrowEvent(ThrowEventData& eventData);
	void doHandleSteerLibraryEvent(ThrowEventData& eventData, bool callbackCalled);
	std::string mThrownEventsParam;
	///@}

	///TypedObject semantics: hardcoded
public:
	static TypeHandle get_class_type()
	{
		return _type_handle;
	}
	static void init_type()
	{
		PandaNode::init_type();
		register_type(_type_handle, "OSSteerVehicle", PandaNode::get_class_type());
	}
	virtual TypeHandle get_type() const
	{
		return get_class_type();
	}
	virtual TypeHandle force_init_type()
	{
		init_type();
		return get_class_type();
	}

private:
	static TypeHandle _type_handle;

};

///inline definitions

inline void OSSteerVehicle::reset()
{
	//
	mVehicle = NULL;
	mSteerPlugInObjectId = ObjectId();
	mInputRadius = 0.0;
	mMovType = OPENSTEER;
	mUpAxisFixed = false;
	mBulletWorld.clear();
	mMaxError = 0.0;
	mDeltaRayDown = mDeltaRayOrig = LVector3f::zero();
	mRayMask = BitMask32::all_off();
	mCorrectHeightRigidBody = 0.0;
	mExternalUpdate = false;
	mPFCallbackCalled = mAOCallbackCalled = mACNCallbackCalled =
			mANCallbackCalled = false;
	mMove = mSteady = mPathFollowing = mAvoidObstacle = mAvoidCloseNeighbor =
			mAvoidNeighbor = ThrowEventData();
	mThrownEventsParam.clear();
}

inline OpenSteer::AbstractVehicle& OSSteerVehicle::getAbstractVehicle()
{
	return *mVehicle;
}

inline OSSteerVehicle::operator OpenSteer::AbstractVehicle&()
{
	return *mVehicle;
}

inline void OSSteerVehicle::enableSteerVehicleEvent(EventThrown event,
		ThrowEventData eventData)
{
	//lock (guard) the mutex
	HOLD_REMUTEX(mMutex)

	doEnableSteerVehicleEvent(event, eventData);
}

inline void OSSteerVehicle::doPathFollowing(const OpenSteer::Vec3& future,
		const OpenSteer::Vec3& onPath, const OpenSteer::Vec3& target,
		const float outside)
{
	//handle Path Following event
	if (mPathFollowing.mEnable)
	{
		doThrowEvent(mPathFollowing);
		//set the flag
		mPFCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doAvoidObstacle(const float minDistanceToCollision)
{
	//handle Avoid Obstacle event
	if (mAvoidObstacle.mEnable)
	{
		doThrowEvent(mAvoidObstacle);
		//set the flag
		mAOCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doAvoidCloseNeighbor(const OpenSteer::AbstractVehicle& other,
		const float additionalDistance)
{
	//handle Avoid Close Neighbor event
	if (mAvoidCloseNeighbor.mEnable)
	{
		doThrowEvent(mAvoidCloseNeighbor);
		//set the flag
		mACNCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doAvoidNeighbor(const OpenSteer::AbstractVehicle& threat,
		const float steer, const OpenSteer::Vec3& ourFuture,
		const OpenSteer::Vec3& threatFuture)
{
	//handle Avoid Neighbor event
	if (mAvoidNeighbor.mEnable)
	{
		doThrowEvent(mAvoidNeighbor);
		//set the flag
		mANCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doThrowEvent(ThrowEventData& eventData)
{
	if (eventData.mThrown)
	{
		eventData.mTimeElapsed += ClockObject::get_global_clock()->get_dt();
		if (eventData.mTimeElapsed >= eventData.mPeriod)
		{
			//enough time is passed: throw the event
			throw_event(eventData.mEventName, EventParameter(this));
			//update elapsed time
			eventData.mTimeElapsed -= eventData.mPeriod;
		}
	}
	else
	{
		//throw the event
		throw_event(eventData.mEventName, EventParameter(this));
		eventData.mThrown = true;
	}
}

inline void OSSteerVehicle::doHandleSteerLibraryEvent(ThrowEventData& eventData, bool callbackCalled)
{
	if (eventData.mEnable)
	{
		if (callbackCalled)
		{
			//event was handled this (or last) frame
			callbackCalled = false;
		}
		else
		{
			//reset event
			if (eventData.mThrown)
			{
				eventData.mThrown = false;
				eventData.mTimeElapsed = 0.0;
			}
		}
	}
}

///Template

class SteerVehicleTemplate: public ComponentTemplate
{
protected:

	virtual SMARTPTR(PandaNode)makeComponent(const ComponentId& compId);

public:
	SteerVehicleTemplate(PandaFramework* pandaFramework,
			WindowFramework* windowFramework);
	virtual ~SteerVehicleTemplate();

	virtual ComponentType componentType() const;
	virtual ComponentFamilyType componentFamilyType() const;

	virtual void setParametersDefaults();

private:

	///TypedObject semantics: hardcoded
public:
	static TypeHandle get_class_type()
	{
		return _type_handle;
	}
	static void init_type()
	{
		ComponentTemplate::init_type();
		register_type(_type_handle, "SteerVehicleTemplate",
				ComponentTemplate::get_class_type());
	}
	virtual TypeHandle get_type() const
	{
		return get_class_type();
	}
	virtual TypeHandle force_init_type()
	{
		init_type();
		return get_class_type();
	}

private:
	static TypeHandle _type_handle;
};

#endif /* OSSTEERVEHICLE_H_ */
