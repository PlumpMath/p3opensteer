/**
 * \file osSteerPlugIn.h
 *
 * \date 2016-05-13
 * \author consultit
 */
#ifndef OSSTEERPLUGIN_H_
#define OSSTEERPLUGIN_H_

#include "osTools.h"
#include "opensteer_includes.h"
#include "nodePath.h"

#ifndef CPPPARSER
#include "library/OpenSteer/PlugIn.h"
#endif //CPPPARSER

class OSSteerVehicle;

/**
 * This class represents a "PlugIns" of the OpenSteer library.
 *
 * \see http://opensteer.sourceforge.net
 *
 * This is a PandaNode.\n
 * Each OSSteerPlugIn component could handle a single pathway and several
 * obstacles.\n
 * The parent node path of this component's object, will be the reference
 * which any SteerVehicle will be reparented to (if necessary) and which any
 * scene computation will be performed wrt.\n
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
class OSSteerPlugIn: public PandaNode
{
PUBLISHED:
	virtual ~OSSteerPlugIn();

	/**
	 * \name PLUGIN
	 */
	///@{
	int setup();
	int cleanup();
	void update(float dt);
	///@}

	/**
	 * \brief Adds a SteerVehicle component to the OpenSteer handling
	 * mechanism.
	 *
	 * If SteerVehicle belongs to any OSSteerPlugIn it is not added.\n
	 * @param steerVehicle The SteerVehicle to add.
	 * @return Result::OK on successful addition, various error conditions otherwise.
	 */
	int addSteerVehicle(PT(OSSteerVehicle)steerVehicle);

	/**
	 * \brief Removes a SteerVehicle component from the OpenSteer handling
	 * mechanism.
	 *
	 * If SteerVehicle doesn't belong to any OSSteerPlugIn it is not removed.\n
	 * @param steerVehicle The SteerVehicle to remove.
	 * @return Result::OK on successful removal, various error conditions otherwise.
	 */
	int removeSteerVehicle(PT(OSSteerVehicle)steerVehicle);

	/**
	 * \brief Sets the pathway of this SteerPlugin.
	 * @param numOfPoints Number of points.
	 * @param points Points' vector.
	 * @param singleRadius Single radius flag.
	 * @param radii Radii' vector.
	 * @param closedCycle Closed cycle flag.
	 */
	void setPathway(int numOfPoints, LPoint3f const points[], bool singleRadius,
			float const radii[], bool closedCycle);

	/**
	 * \brief Adds an OpenSteer obstacle, seen by all SteerPlugins.
	 *
	 * If the object parameter is not NULL,
	 * @param object The Object used as obstacle.
	 * @param type The obstacle type: box, plane, rectangle, sphere.
	 * @param width Obstacle's width (box, rectangle).
	 * @param height Obstacle's height (box, rectangle).
	 * @param depth Obstacle's depth (box).
	 * @param radius Obstacle's radius (sphere).
	 * @param side Obstacle's right side direction.
	 * @param up Obstacle's up direction.
	 * @param forward Obstacle's forward direction.
	 * @param position Obstacle's position.
	 * @param seenFromState Possible values: outside, inside, both.
	 * @return
	 */
	OpenSteer::AbstractObstacle* addObstacle(const NodePath& object,
			const std::string& type, const std::string& seenFromState,
			float width = 0.0, float height = 0.0, float depth = 0.0,
			float radius = 0.0, const LVector3f& side = LVector3f::zero(),
			const LVector3f& up = LVector3f::zero(), const LVector3f& = LVector3f::zero(),
			const LPoint3f& position = LPoint3f::zero());

	/**
	 * \brief Removes an OpenSteer obstacle, seen by all plugins.
	 * @param obstacle The obstacle to remove.
	 */
	void removeObstacle(OpenSteer::AbstractObstacle* obstacle);

	/**
	 * \brief Gets the obstacles, seen by all plugins.
	 * @return The obstacles.
	 */
	OpenSteer::ObstacleGroup getObstacles();

	/**
	 * \name OUTPUT
	 */
	///@{
	void output(ostream &out) const;
	///@}

	/**
	 * \name AbstractPlugIn reference getter & conversion function.
	 */
	///@{
	OpenSteer::AbstractPlugIn& getAbstractPlugIn();
	operator OpenSteer::AbstractPlugIn&();
	///@}

#ifdef OS_DEBUG
	/**
	 * \brief Gets a reference to the OpenSteer Drawer3d Debug node path.
	 * @return The OpenSteer Debug node.
	 */
	NodePath getDrawer3dDebugNodePath() const;
	/**
	 * \brief Gets a reference to the OpenSteer Drawer2d Debug node path.
	 * @return The OpenSteer Debug node.
	 */
	NodePath getDrawer2dDebugNodePath() const;
	/**
	 * \brief Enables/disables debugging.
	 * @param enable True to enable, false to disable.
	 */
	Result debug(bool enable);
#endif

protected:
	friend class OSSteerManager;

	OSSteerPlugIn(const string& name = "SteerPlugIn");

private:
	///Current underlying AbstractPlugIn.
	OpenSteer::AbstractPlugIn* mPlugIn;

	///The reference node path (read only after creation).
	NodePath mReferenceNP;

	///Current time.
	float mCurrentTime;

	///The SteerVehicle components handled by this OSSteerPlugIn.
	std::set<PT(OSSteerVehicle)> mSteerVehicles;

	///The global obstacles handled by all OSSteerPlugIns.
	static OpenSteer::ObstacleGroup mObstacles;
	///The local obstacles handled by this OSSteerPlugIn.
	OpenSteer::ObstacleGroup mLocalObstacles;

	///Unique ref.
	int mRef;

	void do_reset();
	void do_initialize();
	void do_finalize();
	virtual bool initialize();
	virtual void onAddToObjectSetup();
	virtual void onAddToSceneSetup();
	virtual void onRemoveFromObjectCleanup();
	virtual void onRemoveFromSceneCleanup();

	/**
	 * \name Helpers variables/functions.
	 */
	///@{
	void doBuildPathway();
	void doAddObstacles();
	///@}

#ifdef OS_DEBUG
	///OpenSteer debug node paths.
	NodePath mDrawer3dNP, mDrawer2dNP;
	///OpenSteer debug camera.
	NodePath mDebugCamera;
	///OpenSteer DebugDrawers.
	DrawMeshDrawer *mDrawer3d, *mDrawer2d;
	///Enable Debug Draw update.
	bool mEnableDebugDrawUpdate;
#endif

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
