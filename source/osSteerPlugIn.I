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
 * Returns the OSSteerPlugIn type.
 */
INLINE OSSteerPlugIn::OSSteerPlugInType OSSteerPlugIn::get_plug_in_type()
{
	return mPlugInType;
}

/**
 * Resets the OSSteerPlugIn.
 * \note Internal use only.
 */
inline void OSSteerPlugIn::do_reset()
{
	//
	mPlugIn = NULL;
	mPlugInType = ONE_TURNING;
	mReferenceNP.clear();
	mReferenceDebugNP.clear();
	mReferenceDebug2DNP.clear();
	mCurrentTime = 0.0;
	mLocalObstacles.first().clear();
	mLocalObstacles.second().clear();
	mSteerVehicles.clear();
	mPathwayPoints.clear();
	mPathwayRadii.clear();
	mPathwaySingleRadius = mPathwayClosedCycle = false;
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
inline OpenSteer::AbstractPlugIn& OSSteerPlugIn::get_abstract_plug_in()
{
	return *mPlugIn;
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
