/**
 * \file osSteerPlugIn.h
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERPLUGIN_H_
#define OSSTEERPLUGIN_H_

#include "osTools.h"
#include "osSteerManager.h"
#include "opensteer_includes.h"
#include "nodePath.h"

#ifndef CPPPARSER
#include "library/OpenSteer/PlugIn.h"
#include "library/OpenSteer/Obstacle.h"
#endif //CPPPARSER

class OSSteerVehicle;

/**
 * This class represents a "plug-in" of the OpenSteer library.
 *
 * \see http://opensteer.sourceforge.net
 *
 * This PandaNode will create a "steer plug-in".\n
 * Each OSSteerPlugIn object could handle a single pathway and several
 * obstacles.\n
 * An "update" task should call this OSSteerPlugIn's update() method to allow
 * the OSSteerVehicle(s) (simple vehicles), which are added to it, to perform
 * their own steering behaviors.\n
 * \note A OSSteerPlugIn will be reparented to the default reference node on
 * creation (see OSSteerManager).
 *
 * > **OSSteerPlugIn text parameters**:
 * param | type | default | note
 * ------|------|---------|-----
 * | *plugin_type*		|single| *one_turning* | values: one_turning,pedestrian,boid,multiple_pursuit,soccer,capture_the_flag,low_speed_turn,map_drive
 * | *pathway*			|single|"0.0,0.0,0.0:1.0,1.0,1.0$1.0$false" (specified as "p1,py1,pz1:px2,py2,pz2[:...:pxN,pyN,pzN]$r1[:r2:...:rM]$closedCycle" with M,closedCycle=N-1,false,N,true)
 * | *obstacles*  		|multiple| - | each one specified as "objectId1@shape1@seenFromState1[:objectId2@shape2@seenFromState2:...:objectIdN@shapeN@seenFromStateN]"] with shapeX=sphere,box,plane,rectangle and seenFromStateX=outside,inside,both
 *
 * \note parts inside [] are optional.\n
 */
class EXPORT_CLASS OSSteerPlugIn: public PandaNode
{
PUBLISHED:
	/**
	 * Steer Plug-In type.
	 */
	enum OSSteerPlugInType
	{
		ONE_TURNING = 0,
		PEDESTRIAN,
		BOID,
		MULTIPLE_PURSUIT,
		SOCCER,
		CAPTURE_THE_FLAG,
		LOW_SPEED_TURN,
		MAP_DRIVE,
		NONE_PLUGIN
	};

	/**
	 * OSSteerPlugIn proximity database.
	 */
	enum OSProximityDatabase
	{
		LQ_PD,
		BRUTEFORCE_PD
	};

	/**
	 * OSSteerPlugIn playing teams.
	 */
	enum OSPlayingTeam
	{
		TEAM_A,
		TEAM_B,
		NO_TEAM
	};

	virtual ~OSSteerPlugIn();

	/**
	 * \name PLUGIN
	 */
	///@{
	void set_plug_in_type(OSSteerPlugInType type);
	INLINE OSSteerPlugInType get_plug_in_type() const;
	void update(float dt);
	///@}

	/**
	 * \name STEERVEHICLES
	 */
	///@{
	int add_steer_vehicle(NodePath steerVehicleNP);
	int remove_steer_vehicle(NodePath steerVehicleNP);
	bool check_steer_vehicle_compatibility(NodePath steerVehicleNP) const;
	INLINE PT(OSSteerVehicle) get_steer_vehicle(int index) const;
	INLINE int get_num_steer_vehicles() const;
	MAKE_SEQ(get_steer_vehicles, get_num_steer_vehicles, get_steer_vehicle);
	INLINE PT(OSSteerVehicle) operator [](int index) const;
	INLINE int size() const;
	///@}

	/**
	 * \name PATHWAY
	 */
	///@{
	void set_pathway(const ValueList<LPoint3f>& pointList,
			const ValueList<float>& radiusList, bool singleRadius, bool closedCycle);
	INLINE ValueList<LPoint3f> get_pathway_points() const;
	INLINE ValueList<float> get_pathway_radii() const;
	INLINE bool get_pathway_single_radius() const;
	INLINE bool get_pathway_closed_cycle() const;
	///@}

	/**
	 * \name OBSTACLES
	 */
	///@{
	int add_obstacle(NodePath& object, const string& type = string("box"),
			const string& seenFromState = string("both"));
	INLINE int add_obstacle(const string& type = string("box"),
			const string& seenFromState = string("both"),
			float width = 1.0, float height = 1.0, float depth = 1.0, float radius = 1.0,
			const LVector3f& side = LVector3f::unit_x(),
			const LVector3f& up = LVector3f::unit_z(),
			const LVector3f& forward = LVector3f::unit_y(),
			const LPoint3f& position = LPoint3f::zero());
	NodePath remove_obstacle(int ref);
	INLINE int get_obstacle(int index) const;
	INLINE int get_num_obstacles() const;
	MAKE_SEQ(get_obstacles, get_num_obstacles, get_obstacle);
	///@}

	/**
	 * \name PROXIMITY DATABASE SETTINGS (PEDESTRIAN, BOID)
	 */
	///@{
	void set_proximity_database(OSProximityDatabase pd = LQ_PD);
	OSProximityDatabase get_proximity_database() const;
	///@}

	/**
	 * \name WORLD SETTINGS (BOID)
	 */
	///@{
	void set_world_center(const LPoint3f& center);
	LPoint3f get_world_center() const;
	void set_world_radius(float radius);
	float get_world_radius() const;
	///@}

	/**
	 * \name TEAM PLAY SETTINGS (SOCCER)
	 */
	///@{
	int add_player_to_team(PT(OSSteerVehicle) player, OSPlayingTeam team);
	int remove_player_from_team(PT(OSSteerVehicle) player);
	ValueList<LPoint3f> get_playing_field() const;
	void set_playing_field(const LPoint3f& min, const LPoint3f& max,
			float goalFraction = 0.5);
	float get_goal_fraction() const;
	int get_score_team_a() const;
	int get_score_team_b() const;
	///@}

	/**
	 * \name CAPTURE THE FLAG SETTINGS (SOCCER)
	 */
	///@{
	void set_home_base_center(const LPoint3f& center);
	LPoint3f get_home_base_center() const;
	void set_home_base_radius(float radius);
	float get_home_base_radius() const;
	void set_braking_rate(float rate);
	float get_braking_rate() const;
	void set_avoidance_predict_time_min(float time);
	float get_avoidance_predict_time_min() const;
	void set_avoidance_predict_time_max(float time);
	float get_avoidance_predict_time_max() const;
	///@}

	/**
	 * \name STEERING SPEED SETTINGS (LOW_SPEED_TURN)
	 */
	///@{
	void set_steering_speed(float steeringSpeed = 1.0);
	float get_steering_speed() const;
	///@}

	/**
	 * \name OUTPUT
	 */
	///@{
	void output(ostream &out) const;
	///@}

	/**
	 * \name DEBUG DRAWING
	 */
	///@{
	void enable_debug_drawing(NodePath debugCamera);
	void disable_debug_drawing();
	int toggle_debug_drawing(bool enable);
	///@}

public:
	/**
	 * \name C++ ONLY
	 * Library & support low level related methods.
	 */
	///@{
	inline OpenSteer::AbstractPlugIn& get_abstract_plug_in();
	inline operator OpenSteer::AbstractPlugIn&();
	///@}

protected:
	friend class OSSteerManager;

	OSSteerPlugIn(const string& name = "SteerPlugIn");

private:
	///Current underlying AbstractPlugIn.
	OpenSteer::AbstractPlugIn* mPlugIn;
	///The type of this OSSteerPlugIn.
	OSSteerPlugInType mPlugInType;
	///The reference node path.
	NodePath mReferenceNP;
	///The reference node path for debug drawing.
	NodePath mReferenceDebugNP, mReferenceDebug2DNP;
	///Current time.
	float mCurrentTime;
	///Steer vehicles.
	pvector<PT(OSSteerVehicle)> mSteerVehicles;
	///The "local" obstacles handled by this OSSteerPlugIn.
	OSSteerManager::GlobalObstacles mLocalObstacles;
	///Pathway stuff.
	///@{
	ValueList<LPoint3f> mPathwayPoints;
	ValueList<float> mPathwayRadii;
	bool mPathwaySingleRadius, mPathwayClosedCycle;
	///@}

	inline void do_reset();
	void do_initialize();
	void do_finalize();

	/**
	 * \name Helpers variables/functions.
	 */
	///@{
	void do_create_plug_in(OSSteerPlugInType type);
	void do_build_pathway(const string& pathwayParam);
	void do_add_obstacles(const plist<string>& obstacleListParam);
	int do_add_obstacle(NodePath objectNP,
			const string& type, const string& seenFromState,
			float width, float height,	float depth, float radius,
			const LVector3f& side, const LVector3f& up,
			const LVector3f& forward, const LPoint3f& position);
	///@}

	/**
	 * \name SERIALIZATION ONLY SETTINGS.
	 */
	///@{
	// temporary storage for serialized data
	struct SerializedDataTmp
	{
		//pedestrian, boid
		OSProximityDatabase mPD;
		//boid
		LPoint3f mWorldCenter;
		float mWorldRadius;
		//soccer
		LPoint3f mFieldMinPoint, mFieldMaxPoint;
		float mGoalFraction;
		int mScoreTeamA, mScoreTeamB;
		//capture the flag
		LPoint3f mHomeBaseCenter;
		float mHomeBaseRadius, mBrakingRate, mAvoidancePredictTimeMin,
		mAvoidancePredictTimeMax;
		//low speed turn
		float mSteeringSpeed;
	}*mSerializedDataTmpPtr;
	// persistent storage for serialized data
	///@}

#ifdef OS_DEBUG
	///OpenSteer debug node paths.
	NodePath mDrawer3dNP, mDrawer3dStaticNP, mDrawer2dNP;
	///OpenSteer debug camera.
	NodePath mDebugCamera;
	///OpenSteer DebugDrawers.
	ossup::DrawMeshDrawer *mDrawer3d, *mDrawer3dStatic, *mDrawer2d;
	///Enable Debug Draw update.
	bool mEnableDebugDrawUpdate;
	///Draw static geometry
	void do_debug_draw_static_geometry();
#endif

	// Explicitly disabled copy constructor and copy assignment operator.
	OSSteerPlugIn(const OSSteerPlugIn&);
	OSSteerPlugIn& operator=(const OSSteerPlugIn&);

public:
	/**
	 * \name TypedWritable API
	 */
	///@{
	static void register_with_read_factory();
	virtual void write_datagram (BamWriter *manager, Datagram &dg) override;
	virtual int complete_pointers(TypedWritable **p_list, BamReader *manager) override;
	virtual void finalize(BamReader *manager);
	bool require_fully_complete() const;
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
		register_type(_type_handle, "OSSteerPlugIn", PandaNode::get_class_type());
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

INLINE ostream &operator << (ostream &out, const OSSteerPlugIn & plugIn);

///inline
#include "osSteerPlugIn.I"

#endif /* OSSTEERPLUGIN_H_ */
