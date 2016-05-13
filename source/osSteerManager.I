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
 * Returns the number of RNNavMeshes.
 */
INLINE int OSSteerManager::get_num_steer_plug_ins() const
{
	return (int)mNavMeshes.size();
}

/**
 * Returns the number of RNCrowdAgents.
 */
INLINE int OSSteerManager::get_num_steer_vehicles() const
{
	return (int)mCrowdAgents.size();
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
INLINE CollisionTraverser* OSSteerManager::get_collision_traverser()
{
	return mCTrav;
}

/**
 * Get the collision handler.
 */
INLINE CollisionHandlerQueue* OSSteerManager::get_collision_handler()
{
	return mCollisionHandler;
}

/**
 * Get the collision ray.
 */
INLINE CollisionRay* OSSteerManager::get_collision_ray()
{
	return mPickerRay;
}

#endif /* OSSTEERMANGER_I_ */
