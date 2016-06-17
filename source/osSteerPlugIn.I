/**
 * \file osSteerPlugIn.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERPLUGIN_I_
#define OSSTEERPLUGIN_I_


///OSSteerPlugIn inline definitions

/**
 * Resets the OSSteerPlugIn.
 */
inline void OSSteerPlugIn::do_reset()
{
	//
	mPlugIn = NULL;
	mReferenceNP.clear();
	mReferenceDebugNP.clear();
	mReferenceDebug2DNP.clear();
	mCurrentTime = 0.0;
	mSteerVehicles.clear();
#ifdef OS_DEBUG
	mDrawer3dNP.clear();
	mDrawer2dNP.clear();
	mDebugCamera.clear();
	mDrawer3d = mDrawer2d = NULL;
	mEnableDebugDrawUpdate = false;
#endif
}

/**
 * Returns a reference to underlying OpenSteer::AbstractPlugIn.
 */
inline OpenSteer::AbstractPlugIn& OSSteerPlugIn::getAbstractPlugIn()
{
	return *mPlugIn;
}

/**
 *
 */
inline int OSSteerPlugIn::unique_ref()
{
	return ++mRef;
}

/**
 * OpenSteer::AbstractPlugIn reference conversion function.
 */
inline OSSteerPlugIn::operator OpenSteer::AbstractPlugIn&()
{
	return *mPlugIn;
}

INLINE ostream &operator <<(ostream &out, const OSSteerPlugIn& plugIn)
{
	plugIn.output(out);
	return out;
}

#endif /* OSSTEERPLUGIN_I_ */
