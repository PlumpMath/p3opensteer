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

#ifndef CPPPARSER
#include "support/common.h"
#endif //CPPPARSER

/**
 * This class represents a "vehicle" of the RecastNavigation library.
 *
 * \see http://opensteer.sourceforge.net
 *
 * This PandaNode should be added to an OSSteerPlugIn, to perform a "steering
 * behavior".\n
 * A model could be reparented to this OSSteerVehicle.\n
 * An OSSteerVehicle could be of type:
 * - **opensteer** (the default): its movement/orientation follows strictly the
 * path as updated by OpenSteer library
 * - **kinematic**: its movement/orientation is corrected to stand on floor.\n
 * If enabled, this object can throw these events:
 * - on moving (default event name: NODENAME_SteerVehiclet_Move)
 * - on being steady (default event name: NODENAME_SteerVehicle_Steady)
 * - when steering is required to follow a path (default event name:
 *   NODENAME_SteerVehicle_PathFollowing)
 * - when steering is required to avoid an obstacle (default event name:
 *   NODENAME_SteerVehicle_AvoidObstacle)
 * - when steering is required to avoid a close neighbor (i.e. when there is a
 *   collision) (default event name: NODENAME_SteerVehicle_AvoidCloseNeighbor)
 * - when steering is required to avoid a neighbor (i.e. when there is a
 *   potential collision) (default event name:
 *   NODENAME_SteerVehicle_AvoidNeighbor)
 * Events are thrown continuously at a frequency which is the minimum between
 * the fps and the frequency specified (which defaults to 30 times per seconds).
 * \n
 * The argument of each event is a reference to this component.\n
 * \see annotate* SteerLibraryMixin member functions in SteerLibrary.h
 * for more information.
 *
 * \note A OSSteerVehicle will be reparented to the default reference node on
 * creation (see OSSteerManager).
 *
 * > **OSSteerVehicle text parameters**:
 * param | type | default | note
 * ------|------|---------|-----
 * | *thrown_events*			|single| - | specified as "event1@[event_name1]@[frequency1][:...[:eventN@[event_nameN]@[frequencyN]]]" with eventX = move,steady,path_following,avoid_obstacle,avoid_close_neighbor,avoid_neighbor
 * | *type*						|single| *one_turning* | values: one_turning,pedestrian,boid,mp_wanderer,mp_pursuer,player,ball,ctf_seeker,ctf_enemy,low_speed_turn,map_driver
 * | *external_update*			|single| *false* | -
 * | *add_to_plugin*			|single| - | -
 * | *mov_type*					|single| *opensteer* | values: opensteer,kinematic
 * | *up_axis_fixed*			|single| *false* | -
 * | *mass*						|single| 1.0 | -
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
	enum OSSteerVehicleMovType
	{
		OPENSTEER,
		OPENSTEER_KINEMATIC,
		VehicleMovType_NONE
	};

	/**
	 * OSSteerVehicle thrown events.
	 */
	enum OSEventThrown
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
	INLINE void set_settings(const OSVehicleSettings& settings);
	INLINE OSVehicleSettings get_settings();
	INLINE PT(OSSteerPlugIn) get_steer_plug_in() const;
	///@}

	/**
	 * \name EVENTS' CONFIGURATION
	 */
	///@{
	INLINE void enable_steer_vehicle_event(OSEventThrown event, ThrowEventData eventData);
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
	inline OpenSteer::AbstractVehicle& get_abstract_vehicle();
	inline operator OpenSteer::AbstractVehicle&();
	///@}

protected:
	friend class OSSteerManager;
	friend class OSSteerPlugIn;

	OSSteerVehicle(const string& name);

private:
	///The SteerPlugIn this OSSteerVehicle is added to.
	PT(OSSteerPlugIn) mSteerPlugIn;
	///Current underlying OpenSteer Vehicle.
	OpenSteer::AbstractVehicle* mVehicle;
	///The reference node path.
	NodePath mReferenceNP;
	///The movement type.
	OSSteerVehicleMovType mMovType;
	///Flag for up axis fixed (z).
	bool mUpAxisFixed;
	///Height correction for kinematic OSSteerVehicle(s).
	LVector3f mHeigthCorrection;

	inline void do_reset();
	void do_initialize();
	void do_finalize();

	///@{
	void do_update_steer_vehicle(const float currentTime, const float elapsedTime);
	void do_external_update_steer_vehicle(const float currentTime, const float elapsedTime);
	bool mExternalUpdate;
	///@}

	/**
	 * \name SteerLibrary callbacks.
	 */
	///@{
	void do_path_following(const OpenSteer::Vec3& future, const OpenSteer::Vec3& onPath,
			const OpenSteer::Vec3& target, const float outside);
	void do_avoid_obstacle(const float minDistanceToCollision);
	void do_avoid_close_neighbor(const OpenSteer::AbstractVehicle& other, const float additionalDistance);
	void do_avoid_neighbor(const OpenSteer::AbstractVehicle& threat, const float steer,
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
	void do_enable_steer_vehicle_event(OSEventThrown event, ThrowEventData eventData);
	void do_throw_event(ThrowEventData& eventData);
	void do_handle_steer_library_event(ThrowEventData& eventData, bool callbackCalled);
	string mThrownEventsParam;
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
