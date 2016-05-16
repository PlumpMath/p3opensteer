/**
 * \file osSteerManager.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERMANGER_I_
#define OSSTEERMANGER_I_

///OSSteerManager inline definitions

/**
 * Returns the reference NodePath.
 */
INLINE NodePath OSSteerManager::get_reference_node_path() const
{
	return mReferenceNP;
}

/**
 * Sets the reference NodePath.
 * This is usually called after restoring from a bam file.
 */
INLINE void OSSteerManager::set_reference_node_path(const NodePath& reference)
{
	mReferenceNP = reference;
}

/**
 * Returns the reference NodePath for debug drawing.
 * This node should be made sibling of the reference NodePath in the scene.
 */
INLINE NodePath OSSteerManager::get_reference_node_path_debug() const
{
	return mReferenceDebugNP;
}

/**
 * Returns the reference NodePath 2D for debug drawing.
 * This node should be a descendant of a 2D top NodePath of the scene graph (ie
 * aspect2d, render2d, pixel2d).
 */
INLINE NodePath OSSteerManager::get_reference_node_path_debug_2d() const
{
	return mReferenceDebug2DNP;
}

/**
 * Returns the number of OSSteerPlugIns.
 */
INLINE int OSSteerManager::get_num_steer_plug_ins() const
{
	return (int) mSteerPlugIns.size();
}

/**
 * Returns the number of OSSteerVehicles.
 */
INLINE int OSSteerManager::get_num_steer_vehicles() const
{
	return (int) mSteerVehicles.size();
}

/**
 * Returns the obstacle's unique reference (>0) given its index into the list of
 * all defined obstacles, or a negative number on error.
 */
INLINE int OSSteerManager::get_obstacle(int index) const
{
	pvector<ObstacleAttributes> obstacleAttrs = mObstacles.get_second();
	CONTINUE_IF_ELSE_R((index >= 0) && (index < (int ) obstacleAttrs.size()),
			OS_ERROR)

	return obstacleAttrs[index].get_first().get_ref();
}

/**
 * Returns the number of all defined obstacles.
 */
INLINE int OSSteerManager::get_num_obstacles() const
{
	 return (int) mObstacles.get_second().size();
}

/**
 * Get singleton pointer.
 */
INLINE OSSteerManager* OSSteerManager::get_global_ptr()
{
	return Singleton<OSSteerManager>::GetSingletonPtr();
}

/**
 * Get the collide mask.
 */
INLINE CollideMask OSSteerManager::get_collide_mask() const
{
	return mMask;
}

/**
 * Get the collision root.
 */
INLINE NodePath OSSteerManager::get_collision_root() const
{
	return mRoot;
}

/**
 * Get the collision traverser.
 */
INLINE CollisionTraverser* OSSteerManager::get_collision_traverser() const
{
	return mCTrav;
}

/**
 * Get the collision handler.
 */
INLINE CollisionHandlerQueue* OSSteerManager::get_collision_handler() const
{
	return mCollisionHandler;
}

/**
 * Get the collision ray.
 */
INLINE CollisionRay* OSSteerManager::get_collision_ray() const
{
	return mPickerRay;
}

/**
 * Returns a reference to the storage of all obstacles.
 */
inline OSSteerManager::GlobalObstacles& OSSteerManager::get_global_obstacles()
{
	return mObstacles;
}

/**
 *
 */
inline int OSSteerManager::unique_ref()
{
	return ++mRef;
}

#endif /* OSSTEERMANGER_I_ */
