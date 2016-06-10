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
 * Returns the number of RNNavMeshes.
 */
INLINE int OSSteerManager::get_num_steer_plug_ins() const
{
	return (int) mNavMeshes.size();
}

/**
 * Returns the number of RNCrowdAgents.
 */
INLINE int OSSteerManager::get_num_steer_vehicles() const
{
	return (int) mCrowdAgents.size();
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
INLINE CollideMask OSSteerManager::get_collide_mask()
{
	return mMask;
}

/**
 * Get the collision root.
 */
INLINE NodePath OSSteerManager::get_collision_root()
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
