/**
 * \file osSteerVehicle.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERVEHICLE_I_
#define OSSTEERVEHICLE_I_

///OSSteerVehicle inline definitions

inline OpenSteer::AbstractVehicle& OSSteerVehicle::getAbstractVehicle()
{
	return *mVehicle;
}

inline OSSteerVehicle::operator OpenSteer::AbstractVehicle&()
{
	return *mVehicle;
}

inline void OSSteerVehicle::enableSteerVehicleEvent(EventThrown event,
		ThrowEventData eventData)
{
	doEnableSteerVehicleEvent(event, eventData);
}

inline void OSSteerVehicle::doPathFollowing(const OpenSteer::Vec3& future,
		const OpenSteer::Vec3& onPath, const OpenSteer::Vec3& target,
		const float outside)
{
	//handle Path Following event
	if (mPathFollowing.mEnable)
	{
		doThrowEvent(mPathFollowing);
		//set the flag
		mPFCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doAvoidObstacle(const float minDistanceToCollision)
{
	//handle Avoid Obstacle event
	if (mAvoidObstacle.mEnable)
	{
		doThrowEvent(mAvoidObstacle);
		//set the flag
		mAOCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doAvoidCloseNeighbor(const OpenSteer::AbstractVehicle& other,
		const float additionalDistance)
{
	//handle Avoid Close Neighbor event
	if (mAvoidCloseNeighbor.mEnable)
	{
		doThrowEvent(mAvoidCloseNeighbor);
		//set the flag
		mACNCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doAvoidNeighbor(const OpenSteer::AbstractVehicle& threat,
		const float steer, const OpenSteer::Vec3& ourFuture,
		const OpenSteer::Vec3& threatFuture)
{
	//handle Avoid Neighbor event
	if (mAvoidNeighbor.mEnable)
	{
		doThrowEvent(mAvoidNeighbor);
		//set the flag
		mANCallbackCalled = true;
	}
}

inline void OSSteerVehicle::doThrowEvent(ThrowEventData& eventData)
{
	if (eventData.mThrown)
	{
		eventData.mTimeElapsed += ClockObject::get_global_clock()->get_dt();
		if (eventData.mTimeElapsed >= eventData.mPeriod)
		{
			//enough time is passed: throw the event
			throw_event(eventData.mEventName, EventParameter(this));
			//update elapsed time
			eventData.mTimeElapsed -= eventData.mPeriod;
		}
	}
	else
	{
		//throw the event
		throw_event(eventData.mEventName, EventParameter(this));
		eventData.mThrown = true;
	}
}

inline void OSSteerVehicle::doHandleSteerLibraryEvent(ThrowEventData& eventData, bool callbackCalled)
{
	if (eventData.mEnable)
	{
		if (callbackCalled)
		{
			//event was handled this (or last) frame
			callbackCalled = false;
		}
		else
		{
			//reset event
			if (eventData.mThrown)
			{
				eventData.mThrown = false;
				eventData.mTimeElapsed = 0.0;
			}
		}
	}
}

/**
 * Resets OSSteerVehicle.
 */
inline void OSSteerVehicle::do_reset()
{
	//
	mVehicle = NULL;
//	mSteerPlugInObjectId = ObjectId();
	mInputRadius = 0.0;
	mMovType = OPENSTEER;
	mUpAxisFixed = false;
//	mBulletWorld.clear();
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

INLINE ostream &operator <<(ostream &out, const OSSteerVehicle& steerVehicle)
{
	steerVehicle.output(out);
	return out;
}

#endif /* OSSTEERVEHICLE_I_ */
