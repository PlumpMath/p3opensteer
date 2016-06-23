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
 * Returns the OSSteerVehicle given its index, or NULL on error.
 */
INLINE PT(OSSteerVehicle) OSSteerPlugIn::get_steer_vehicle(int index) const
{
	CONTINUE_IF_ELSE_R((index >= 0) && (index < (int)mSteerVehicles.size()), NULL)

	return mSteerVehicles[index];
}

/**
 * Returns the number of OSSteerVehicles added to this OSSteerPlugIn.
 */
INLINE int OSSteerPlugIn::get_num_steer_vehicles() const
{
	return (int)mSteerVehicles.size();
}

/**
 * Returns the OSSteerVehicle given its index.
 */
INLINE PT(OSSteerVehicle) OSSteerPlugIn::operator [](int index) const
{
	return get_steer_vehicle(index);
}

/**
 * Returns the number of OSSteerVehicles added to this OSSteerPlugIn.
 */
INLINE int OSSteerPlugIn::size() const
{
	return get_num_steer_vehicles();
}

/**
 * Returns the pathway points.
 */
INLINE ValueList<LPoint3f> OSSteerPlugIn::get_pathway_points()
{
	return mPathwayPoints;
}

/**
 * Returns the pathway points.
 */
INLINE ValueList<float> OSSteerPlugIn::get_pathway_radii()
{
	return mPathwayRadii;
}

/**
 * Returns the pathway points.
 */
INLINE bool OSSteerPlugIn::get_pathway_single_radius()
{
	return mPathwaySingleRadius;
}

/**
 * Returns the pathway points.
 */
INLINE bool OSSteerPlugIn::get_pathway_closed_cycle()
{
	return mPathwayClosedCycle;
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
	mBuildFromBam = false;
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
