/**
 * \file osSteerVehicle.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERVEHICLE_I_
#define OSSTEERVEHICLE_I_

///OSSteerVehicle inline definitions

/**
 * Returns the OSSteerVehicle type.
 */
INLINE OSSteerVehicle::OSSteerVehicleType OSSteerVehicle::get_vehicle_type()
{
	return mVehicleType;
}

/**
 * Enables/disables OSSteerVehicle's up axis fixed.
 * Returns the value actually set.
 */
INLINE bool OSSteerVehicle::enable_up_axis_fixed(bool enable)
{
	mUpAxisFixed = enable;
	return mUpAxisFixed;
}

/**
 * Sets the OSSteerVehicle's settings.
 */
INLINE void OSSteerVehicle::set_settings(const OSVehicleSettings& settings)
{
	//save a copy
	mVehicleSettings = settings;
	//set OpenSteer vehicle settings
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(mVehicleSettings);
}

/**
 * Returns OSSteerVehicle's settings.
 */
INLINE OSVehicleSettings OSSteerVehicle::get_settings()
{
	//get vehicle settings
	return mVehicleSettings;
}

/**
 * Sets the OSSteerVehicle's move type (opensteer native or kinematic).
 */
INLINE void OSSteerVehicle::set_mov_type(OSSteerVehicle::OSSteerVehicleMovType movType)
{
	//if there isn't a traverser only OPENSTEER is allowed
	OSSteerManager::GetSingletonPtr()->get_collision_traverser() ?
			mMovType = movType : mMovType = OPENSTEER;
}

/**
 * Returns OSSteerVehicle's move type (opensteer native or kinematic).
 */
INLINE OSSteerVehicle::OSSteerVehicleMovType OSSteerVehicle::get_mov_type()
{
	return mMovType;
}

/**
 * Returns the OSSteerPlugIn this OSSteerVehicle is added to.
 */
INLINE PT(OSSteerPlugIn) OSSteerVehicle::get_steer_plug_in() const
{
	return mSteerPlugIn;
}

/**
 * Enables/disables the OSSteerVehicle's events to be thrown.
 */
INLINE void OSSteerVehicle::enable_steer_vehicle_event(OSEventThrown event,
		ThrowEventData eventData)
{
	do_enable_steer_vehicle_event(event, eventData);
}

/**
 * Resets OSSteerVehicle.
 * \note Internal use only.
 */
inline void OSSteerVehicle::do_reset()
{
	//
	mVehicle = NULL;
	mVehicleType = ONE_TURNING;
	mMovType = OPENSTEER;
	mSteerPlugIn.clear();
	mReferenceNP.clear();
	mVehicleSettings = OSVehicleSettings();
	mHeigthCorrection = LVector3f::zero();
	mUpAxisFixed = false;
	mExternalUpdate = false;
	mPFCallbackCalled = mAOCallbackCalled = mACNCallbackCalled =
			mANCallbackCalled = false;
	mMove = mSteady = mPathFollowing = mAvoidObstacle = mAvoidCloseNeighbor =
			mAvoidNeighbor = ThrowEventData();
}

/**
 * Returns a reference to underlying OpenSteer::AbstractVehicle.
 */
inline OpenSteer::AbstractVehicle& OSSteerVehicle::get_abstract_vehicle()
{
	return *mVehicle;
}

/**
 * AbstractVehicle reference conversion function.
 */
inline OSSteerVehicle::operator OpenSteer::AbstractVehicle&()
{
	return *mVehicle;
}

INLINE ostream &operator <<(ostream &out, const OSSteerVehicle& steerVehicle)
{
	steerVehicle.output(out);
	return out;
}

#endif /* OSSTEERVEHICLE_I_ */
