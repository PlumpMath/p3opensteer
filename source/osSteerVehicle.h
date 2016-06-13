/**
 * \file osSteerVehicle.h
 *
 * \date 2016-05-13
 * \author consultit
 */
#ifndef OSSTEERVEHICLE_H_
#define OSSTEERVEHICLE_H_

#include "osSteerPlugIn.h"
#include "osSteerManager.h"
#include "osTools.h"
#include "opensteer_includes.h"
#include "nodePath.h"
#include "throw_event.h"

#ifndef CPPPARSER
#include "support/common.h"
#endif //CPPPARSER

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
 *
 * \note parts inside [] are optional.\n
 */
class EXPORT_CLASS OSSteerVehicle: public PandaNode
{
PUBLISHED:
	/**
	 * OSSteerVehicle movement type.
	 */
	enum SteerVehicleMovType
	{
		OPENSTEER,
		OPENSTEER_KINEMATIC,
		VehicleMovType_NONE
	};

	/**
	 * OSSteerVehicle thrown events.
	 */
	enum EventThrown
	{
		MOVEEVENT,              //!< MOVEEVENT
		STEADYEVENT,            //!< STEADYEVENT
		PATHFOLLOWINGEVENT,     //!< PATHFOLLOWINGEVENT
		AVOIDOBSTACLEEVENT,     //!< AVOIDOBSTACLEEVENT
		AVOIDCLOSENEIGHBOREVENT,//!< AVOIDCLOSENEIGHBOREVENT
		AVOIDNEIGHBOREVENT      //!< AVOIDNEIGHBOREVENT
	};

	virtual ~OSSteerVehicle();

	/**
	 * \name CONFIGURATION SETTINGS
	 */
	///@{
	INLINE void setSettings(const OSVehicleSettings& settings);
	INLINE OSVehicleSettings getSettings();
	/**
	 * \name Get the SteerPlugIn owner object.
	 *
	 * \return The SteerPlugIn object.
	 */
	INLINE PT(OSSteerPlugIn) getSteerPlugIn() const;
	///@}

	/**
	 * \name EVENTS' CONFIGURATION
	 */
	///@{
	/**
	 * \brief Enables/disables the OSSteerVehicle event to be thrown.
	 * @param event The OSSteerVehicle event.
	 * @param eventData The OSSteerVehicle event data. ThrowEventData::mEnable
	 * will enable/disable the event.
	 */
	INLINE void enableSteerVehicleEvent(EventThrown event, ThrowEventData eventData);
	///@}

	/**
	 * \name OUTPUT
	 */
	///@{
	void output(ostream &out) const;
	///@}

public:
	/**
	 * \name C++ ONLY
	 * Library & support low level related methods.
	 */
	///@{
	/**
	 * AbstractVehicle reference getter & conversion function.
	 */
	inline OpenSteer::AbstractVehicle& getAbstractVehicle();
	inline operator OpenSteer::AbstractVehicle&();
	///@}

protected:
	friend class OSSteerManager;
	friend class OSSteerPlugIn;

	OSSteerVehicle(const string& name);

private:
	///Current underlying Vehicle.
	OpenSteer::AbstractVehicle* mVehicle;
	///The SteerPlugIn owner object.
	PT(OSSteerPlugIn) mSteerPlugIn;
	///The reference node path.
	NodePath mReferenceNP;
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
//	SMARTPTR(BulletWorld) mBulletWorld;
//	float mMaxError;
//	LVector3f mDeltaRayDown, mDeltaRayOrig;
//	BulletClosestHitRayResult mHitResult;
//	BitMask32 mRayMask;
	float mCorrectHeightRigidBody;
	///@}

	void do_reset();
	void do_initialize();
	void do_finalize();

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

	// Explicitly disabled copy constructor and copy assignment operator.
	OSSteerVehicle(const OSSteerVehicle&);
	OSSteerVehicle& operator=(const OSSteerVehicle&);

public:
	/**
	 * \name TypedWritable API
	 */
	///@{
	static void register_with_read_factory();
	virtual void write_datagram(BamWriter *manager, Datagram &dg) override;
	virtual int complete_pointers(TypedWritable **plist, BamReader *manager) override;
	///@}

protected:
	static TypedWritable *make_from_bam(const FactoryParams &params);
	virtual void fillin(DatagramIterator &scan, BamReader *manager) override;

public:
	/**
	 * \name TypedObject API
	 */
	///@{
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
	///@}

private:
	static TypeHandle _type_handle;

};

INLINE ostream &operator << (ostream &out, const OSSteerVehicle & crowdAgent);

//VehicleAddOn typedef.
typedef ossup::VehicleAddOnMixin<ossup::SimpleVehicle, OSSteerVehicle> VehicleAddOn;

///inline
#include "osSteerVehicle.I"

#endif /* OSSTEERVEHICLE_H_ */
