/**
 * \file osSteerVehicle.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERVEHICLE_I_
#define OSSTEERVEHICLE_I_

///OSSteerVehicle inline definitions

INLINE void OSSteerVehicle::enableSteerVehicleEvent(EventThrown event,
		ThrowEventData eventData)
{
	doEnableSteerVehicleEvent(event, eventData);
}

INLINE void OSSteerVehicle::setSettings(const OSVehicleSettings& settings)
{
	//set vehicle settings
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
}

INLINE OSVehicleSettings OSSteerVehicle::getSettings()
{
	//get vehicle settings
	return static_cast<VehicleAddOn*>(mVehicle)->getSettings();
}

INLINE PT(OSSteerPlugIn) OSSteerVehicle::getSteerPlugIn() const
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
	mInputRadius = 0.0;
	mMovType = OPENSTEER;
	mUpAxisFixed = false;
//	mBulletWorld.clear(); XXX
//	mMaxError = 0.0;
//	mDeltaRayDown = mDeltaRayOrig = LVector3f::zero();
//	mRayMask = BitMask32::all_off();
//	mCorrectHeightRigidBody = 0.0;
	mExternalUpdate = false;
	mPFCallbackCalled = mAOCallbackCalled = mACNCallbackCalled =
			mANCallbackCalled = false;
	mMove = mSteady = mPathFollowing = mAvoidObstacle = mAvoidCloseNeighbor =
			mAvoidNeighbor = ThrowEventData();
	mThrownEventsParam.clear();
}

inline OpenSteer::AbstractVehicle& OSSteerVehicle::getAbstractVehicle()
{
	return *mVehicle;
}

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
