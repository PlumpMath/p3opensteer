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
 * This PandaNode will create a "plug-in".\n
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
 * | *type*				|single| *one_turning* | values: one_turning,pedestrian,boid,multiple_pursuit,soccer,capture_the_flag,low_speed_turn,map_drive
 * | *pathway*			|single|"0.0,0.0,0.0:1.0,1.0,1.0$1.0$false" (specified as "p1,py1,pz1:px2,py2,pz2[:...:pxN,pyN,pzN]$r1[:r2:...:rM]$closedCycle" with M,closedCycle=N-1,false,N,true)
 * | *obstacles*  		|multiple| - | each one specified as "objectId1@shape1@seenFromState1[:objectId2@shape2@seenFromState2:...:objectIdN@shapeN@seenFromStateN]"] with shapeX=sphere,box,plane,rectangle and seenFromStateX=outside,inside,both
 *
 * \note parts inside [] are optional.\n
 */
class EXPORT_CLASS OSSteerPlugIn: public PandaNode
{
public:
//	typedef Pair<pvector<LPoint3f>, > ObstacleAttributes;
//	typedef Pair<OpenSteer::ObstacleGroup, pvector<ObstacleAttributes> > GlobalObstacles;

PUBLISHED:
	virtual ~OSSteerPlugIn();

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
	 * \name PLUGIN
	 */
	///@{
	INLINE OSSteerPlugInType get_plug_in_type();
	void update(float dt);
	///@}

	/**
	 * \name STEERVEHICLES
	 */
	///@{
	int add_steer_vehicle(NodePath steerVehicleNP);
	int remove_steer_vehicle(NodePath steerVehicleNP);
	INLINE PT(OSSteerVehicle) get_steer_vehicle(int index) const; //XXX
	INLINE int get_num_steer_vehicles() const; //XXX
	MAKE_SEQ(get_steer_vehicles, get_num_steer_vehicles, get_steer_vehicle); //XXX
	INLINE PT(OSSteerVehicle) operator [](int index) const; //XXX
	INLINE int size() const; //XXX
	///@}

	/**
	 * \name PATHWAY
	 */
	///@{
	void set_pathway(const ValueList<LPoint3f>& pointList,
			const ValueList<float>& radiusList, bool singleRadius, bool closedCycle);
	///@}

	/**
	 * \name OBSTACLES
	 */
	///@{
	int add_obstacle(NodePath& object,
			const string& type, const string& seenFromState,
			float width = 0.0, float height = 0.0, float depth = 0.0,
			float radius = 0.0, const LVector3f& side = LVector3f::zero(),
			const LVector3f& up = LVector3f::zero(), const LVector3f& = LVector3f::zero(),
			const LPoint3f& position = LPoint3f::zero());
	NodePath remove_obstacle(int ref);
	///@}

	/**
	 * \name LOW SPEED TURN SETTINGS.
	 */
	///@{
	void set_steering_speed(float steeringSpeed);
	float get_steering_speed();
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
	void do_build_pathway(const string& pathwayParam);
	void do_add_obstacles(const plist<string>& obstacleListParam);
	///@}

#ifdef OS_DEBUG
	///OpenSteer debug node paths.
	NodePath mDrawer3dNP, mDrawer2dNP;
	///OpenSteer debug camera.
	NodePath mDebugCamera;
	///OpenSteer DebugDrawers.
	ossup::DrawMeshDrawer *mDrawer3d, *mDrawer2d;
	///Enable Debug Draw update.
	bool mEnableDebugDrawUpdate;
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
	void write_plug_in_datagram(OSSteerPlugInType type, Datagram &dg) const;
	void read_plug_in_datagram(OSSteerPlugInType type, DatagramIterator &scan);

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
