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
 * Returns the number of RNNavMeshes.
 */
INLINE int OSSteerManager::get_num_steer_plug_ins() const
{
	return (int) mSteerPlugIns.size();
}

/**
 * Returns the number of RNCrowdAgents.
 */
INLINE int OSSteerManager::get_num_steer_vehicles() const
{
	return (int) mSteerVehicles.size();
}

/**
 * Get singleton pointer.
 */
INLINE OSSteerManager* OSSteerManager::get_global_ptr()
{
	return Singleton<OSSteerManager>::GetSingletonPtr();
}

/**
 * Returns a reference to the storage of all obstacles.
 */
inline pvector<OSSteerManager::ObstacleAttributes>& OSSteerManager::get_opensteer_obstacles()
{
	return mOpensteerObstacles;
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

#endif /* OSSTEERMANGER_I_ */
