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
#include "support/PlugIn_CaptureTheFlag.h"
#endif //CPPPARSER

/**
 * This class represents a "vehicle" of the OpenSteer library.
 *
 * \see http://opensteer.sourceforge.net
 *
 * This PandaNode will create a "steer vehicle"and should be added to an
 * OSSteerPlugIn, to perform its "steering behavior".\n
 * A model could be reparented to this OSSteerVehicle.\n
 * An OSSteerVehicle could be of type:
 * - **opensteer** (the default): its movement follows strictly the path as
 *   updated by OpenSteer library
 * - **kinematic**: its movement is corrected to stand on floor.\n
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
 * | *vehicle_type*				|single| *one_turning* | values: one_turning,pedestrian,boid,mp_wanderer,mp_pursuer,player,ball,ctf_seeker,ctf_enemy,low_speed_turn,map_driver
 * | *mov_type*					|single| *opensteer* | values: opensteer,kinematic
 * | *add_to_plugin*			|single| - | -
 * | *mass*						|single| 1.0 | -
 * | *speed*					|single| 0.0 | -
 * | *max_force*				|single| 0.1 | -
 * | *max_speed*				|single| 1.0 | -
 * | *up_axis_fixed*			|single| *false* | -
 * | *up_axis_fixed_mode*		|single| *light* | valid when up_axis_fixed = true; values: light,medium,strong
 * | *external_update*			|single| *false* | -
 *
 * \note parts inside [] are optional.\n
 */
class EXPORT_CLASS OSSteerVehicle: public PandaNode
{
PUBLISHED:
	/**
	 * Steer Vehicle type.
	 */
	enum OSSteerVehicleType
	{
		ONE_TURNING = 0,
		PEDESTRIAN,
		BOID,
		MP_WANDERER,
		MP_PURSUER,
		PLAYER,
		BALL,
		CTF_SEEKER,
		CTF_ENEMY,
		LOW_SPEED_TURN,
		MAP_DRIVER,
		NONE_VEHICLE
	};

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
	 * OSSteerVehicle up axis fixed mode.
	 */
	enum OSSteerVehicleUpAxisFixedMode
	{
		UP_AXIS_FIXED_LIGHT,
		UP_AXIS_FIXED_MEDIUM,
		UP_AXIS_FIXED_STRONG
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

	/**
	 * PEDESTRIAN OSSteerVehicle path direction.
	 */
	enum OSPathDirection
	{
		UPSTREAM,
		DOWNSTREAM
	};

	/**
	 * CTF_SEEKER OSSteerVehicle state.
	 */
	enum OSSeekerState
	{
#ifndef CPPPARSER
		RUNNING = ossup::CtfSeeker<OSSteerVehicle>::running,
		TAGGED = ossup::CtfSeeker<OSSteerVehicle>::tagged,
		ATGOAL = ossup::CtfSeeker<OSSteerVehicle>::atGoal
#else
		RUNNING,TAGGED,ATGOAL
#endif //CPPPARSER

	};

	virtual ~OSSteerVehicle();

	/**
	 * \name VEHICLE
	 */
	///@{
	void set_vehicle_type(OSSteerVehicleType type);
	INLINE OSSteerVehicleType get_vehicle_type() const;
	INLINE void set_mov_type(OSSteerVehicleMovType movType);
	INLINE OSSteerVehicleMovType get_mov_type() const;
	///@}

	/**
	 * \name COMMON CONFIGURATION SETTINGS
	 */
	///@{
	INLINE void set_settings(const OSVehicleSettings& settings);
	INLINE OSVehicleSettings get_settings() const;
	INLINE void set_mass(float mass);
	INLINE float get_mass() const;
	INLINE void set_radius(float radius);
	INLINE float get_radius() const;
	INLINE void set_speed(float speed);
	INLINE float get_speed() const;
	INLINE void set_max_force(float maxForce);
	INLINE float get_max_force() const;
	INLINE void set_max_speed(float maxSpeed);
	INLINE float get_max_speed() const;
	INLINE void set_forward(const LVector3f& forward);
	INLINE LVector3f get_forward() const;
	INLINE void set_side(const LVector3f& side);
	INLINE LVector3f get_side() const;
	INLINE void set_up(const LVector3f& up);
	INLINE LVector3f get_up() const;
	INLINE void set_position(const LPoint3f& position);
	INLINE LPoint3f get_position() const;
	void set_external_update(bool enable);
	INLINE bool get_external_update() const;
	INLINE void set_up_axis_fixed(bool enable);
	INLINE bool get_up_axis_fixed() const;
	INLINE void set_up_axis_fixed_mode(OSSteerVehicleUpAxisFixedMode mode);
	INLINE OSSteerVehicleUpAxisFixedMode get_up_axis_fixed_mode() const;
	INLINE PT(OSSteerPlugIn) get_steer_plug_in() const;
	///@}

	/**
	 * \name PATHWAY DIRECTION SETTINGS (PEDESTRIAN, MAP_DRIVER)
	 */
	///@{
	void set_pathway_direction(OSPathDirection direction);
	OSPathDirection get_pathway_direction() const;
	///@}

	/**
	 * \name WALK MOVEMENT SETTINGS (PEDESTRIAN)
	 */
	///@{
	void set_reverse_at_end_point(bool enable = false);
	bool get_reverse_at_end_point() const;
	void set_wander_behavior(bool enable = false);
	bool get_wander_behavior() const;
	void set_pathway_end_points(const ValueList<int>& points);
	ValueList<int> get_pathway_end_points() const;
	///@}

	/**
	 * \name FLOCK SETTINGS (BOID)
	 */
	///@{
	void set_flock_settings(const OSFlockSettings& settings);
	OSFlockSettings get_flock_settings() const;
	///@}

	/**
	 * \name TEAM PLAY SETTINGS (PLAYER)
	 */
	///@{
	OSSteerPlugIn::OSPlayingTeam get_playing_team() const;
	void set_playing_distance(float distance);
	float get_playing_distance() const;
	///@}

	/**
	 * \name SEEKER STATE SETTINGS (CTF_SEEKER)
	 */
	///@{
	OSSeekerState get_seeker_state() const;
	///@}

	/**
	 * \name MAP STEERING SETTINGS (MAP_DRIVER)
	 */
	///@{
	void set_base_look_ahead_time(float time = 3.0);
	float get_base_look_ahead_time() const;
	void set_incremental_steering(bool enable = true);
	bool get_incremental_steering() const;
	void set_map_prediction_type(OSSteerPlugIn::OSMapPredictionType
			type = OSSteerPlugIn::CURVED_PREDICTION);
	OSSteerPlugIn::OSMapPredictionType get_map_prediction_type() const;
	///@}

	/**
	 * \name STEERING SPEED SETTINGS (LOW_SPEED_TURN)
	 */
	///@{
	void set_steering_speed(float speed = 1.0);
	float get_steering_speed() const;
	///@}

	/**
	 * \name EVENTS' CONFIGURATION
	 */
	///@{
	INLINE void enable_steer_vehicle_event(OSEventThrown event,
			ThrowEventData eventData);
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
	///Current underlying OpenSteer Vehicle.
	OpenSteer::AbstractVehicle* mVehicle;
	///The type of this OSSteerPlugIn.
	OSSteerVehicleType mVehicleType;
	///The movement type of this OSSteerPlugIn.
	OSSteerVehicleMovType mMovType;
	///The OSSteerPlugIn this OSSteerVehicle is added to.
	PT(OSSteerPlugIn) mSteerPlugIn;
	///The reference node path.
	NodePath mReferenceNP;
	///OSSteerVehicle settings.
	OSVehicleSettings mVehicleSettings;
	///Height correction for kinematic OSSteerVehicle(s).
	LVector3f mHeigthCorrection;
	///Flag for up axis fixed (z).
	bool mUpAxisFixed;
	///Up axis fixed mode
	OSSteerVehicleUpAxisFixedMode mUpAxisFixedMode;

	inline void do_reset();
	void do_initialize();
	void do_finalize();

	/**
	 * \name Helpers variables/functions.
	 */
	///@{
	void do_create_vehicle(OSSteerVehicleType type);
	void do_update_steer_vehicle(const float currentTime, const float elapsedTime);
	void do_external_update_steer_vehicle(const float currentTime, const float elapsedTime);
	///External update.
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
	///@}

	/**
	 * \name SERIALIZATION ONLY SETTINGS.
	 */
	///@{
	// temporary storage for serialized data
	struct SerializedDataTmp
	{
		//pedestrian
		bool mReverseAtEndPoint;
		bool mWanderBehavior;
		ValueList<int> mPathwayEndPointIdx;
		//pedestrian, map driver
		OSPathDirection mPathwayDirection;
		//boid
		OSFlockSettings mFlockSettings;
		//player
		LPoint3f mHome;
		float mDistHomeToBall;
		//ctf seeker, ctf enemy
		bool mAvoiding;
		//ctf seeker
		bool mEvading;
		float mLastRunningTime;
		ossup::CtfBase<OSSteerVehicle>::seekerState mState;
		//low speed turn
		float mSteeringSpeed;
		//map driver
		float mBaseLookAheadTime;
		bool mIncrementalSteering;
		OSSteerPlugIn::OSMapPredictionType mMapPredictionType;
		LVector3f mCurrentSteering, mQqqLastNearestObstacle;
		bool mQQQoaJustScraping, mStuck;
		float mHalfWidth, mHalfLength, mDynamicRadius;
	}*mSerializedDataTmpPtr;
	// persistent storage for serialized data
	//player
	OSSteerPlugIn::OSPlayingTeam mPlayingTeam_ser;
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
	virtual void finalize(BamReader *manager);
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

INLINE ostream &operator << (ostream &out, const OSSteerVehicle & steerVehicle);

//VehicleAddOn typedef.
typedef ossup::VehicleAddOnMixin<ossup::SimpleVehicle, OSSteerVehicle> VehicleAddOn;

///inline
#include "osSteerVehicle.I"

#endif /* OSSTEERVEHICLE_H_ */
