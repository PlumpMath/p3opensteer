/**
 * \file osSteerPlugIn.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERPLUGIN_I_
#define OSSTEERPLUGIN_I_


///OSSteerPlugIn inline definitions

INLINE OpenSteer::ObstacleGroup OSSteerPlugIn::getObstacles()
{
	return mObstacles;
}

INLINE OpenSteer::AbstractPlugIn& OSSteerPlugIn::getAbstractPlugIn()
{
	return *mPlugIn;
}

INLINE OSSteerPlugIn::operator OpenSteer::AbstractPlugIn&()
{
	return *mPlugIn;
}

#ifdef OS_DEBUG
INLINE NodePath OSSteerPlugIn::getDrawer3dDebugNodePath() const
{
	return mDrawer3dNP;
}

INLINE NodePath OSSteerPlugIn::getDrawer2dDebugNodePath() const
{
	return mDrawer2dNP;
}
#endif //OS_DEBUG

inline void OSSteerPlugIn::do_reset()
{
	//
	mPlugIn = NULL;
	mReferenceNP = NodePath();
	mCurrentTime = 0.0;
	mSteerVehicles.clear();
#ifdef OS_DEBUG
	mDrawer3dNP = NodePath();
	mDrawer2dNP = NodePath();
	mDebugCamera = NodePath();
	mDrawer3d = mDrawer2d = NULL;
	mEnableDebugDrawUpdate = false;
#endif
}

INLINE ostream &operator <<(ostream &out, const OSSteerPlugIn& pluIn)
{
	pluIn.output(out);
	return out;
}

#endif /* OSSTEERPLUGIN_I_ */
