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
 * Enables/disables the OSSteerVehicle's events to be thrown.
 */
INLINE void OSSteerVehicle::enable_steer_vehicle_event(OSEventThrown event,
		ThrowEventData eventData)
{
	do_enable_steer_vehicle_event(event, eventData);
}

/**
 * Sets the OSSteerVehicle's settings.
 */
INLINE void OSSteerVehicle::set_settings(const OSVehicleSettings& settings)
{
	//set vehicle settings
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
}

/**
 * Returns OSSteerVehicle's settings.
 */
INLINE OSVehicleSettings OSSteerVehicle::get_settings()
{
	//get vehicle settings
	return static_cast<VehicleAddOn*>(mVehicle)->getSettings();
}

/**
 * Returns the OSSteerPlugIn this OSSteerVehicle is added to.
 */
INLINE WPT(OSSteerPlugIn) OSSteerVehicle::get_steer_plug_in() const
{
	return mSteerPlugIn;
}

/**
 * Resets OSSteerVehicle.
 */
inline void OSSteerVehicle::do_reset()
{
	//
	mVehicle = NULL;
	mSteerPlugIn.clear();
	mMovType = OPENSTEER;
	mUpAxisFixed = false;
	mHeigthCorrection = LVector3f::zero();
	mExternalUpdate = false;
	mPFCallbackCalled = mAOCallbackCalled = mACNCallbackCalled =
			mANCallbackCalled = false;
	mMove = mSteady = mPathFollowing = mAvoidObstacle = mAvoidCloseNeighbor =
			mAvoidNeighbor = ThrowEventData();
	mThrownEventsParam.clear();
}

/**
 * Returns a reference to underlying AbstractVehicle.
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
