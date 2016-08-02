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
INLINE OSSteerVehicle::OSSteerVehicleType OSSteerVehicle::get_vehicle_type() const
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
 * Sets the OSSteerVehicle's settings as a whole.
 */
INLINE void OSSteerVehicle::set_settings(const OSVehicleSettings& settings)
{
	//set OpenSteer vehicle settings
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
}

/**
 * Returns OSSteerVehicle's settings as a whole.
 */
INLINE OSVehicleSettings OSSteerVehicle::get_settings() const
{
	//get OpenSteer vehicle settings
	return static_cast<VehicleAddOn*>(mVehicle)->getSettings();
}

/**
 * Sets the OSSteerVehicle's mass.
 */
INLINE void OSSteerVehicle::set_mass(float mass)
{
	mVehicle->setMass(mass);
}

/**
 * Returns the OSSteerVehicle's mass.
 */
INLINE float OSSteerVehicle::get_mass() const
{
	return mVehicle->mass();
}

/**
 * Sets the OSSteerVehicle's radius.
 */
INLINE void OSSteerVehicle::set_radius(float radius)
{
	mVehicle->setRadius(radius);
}

/**
 * Returns the OSSteerVehicle's radius.
 */
INLINE float OSSteerVehicle::get_radius() const
{
	return mVehicle->radius();
}

/**
 * Sets the OSSteerVehicle's speed.
 */
INLINE void OSSteerVehicle::set_speed(float speed)
{
	mVehicle->setSpeed(speed);
}

/**
 * Returns the OSSteerVehicle's speed.
 */
INLINE float OSSteerVehicle::get_speed() const
{
	return mVehicle->speed();
}

/**
 * Sets the OSSteerVehicle's max force.
 */
INLINE void OSSteerVehicle::set_max_force(float maxForce)
{
	mVehicle->setMaxForce(maxForce);
}

/**
 * Returns the OSSteerVehicle's max force.
 */
INLINE float OSSteerVehicle::get_max_force() const
{
	return mVehicle->maxForce();
}

/**
 * Sets the OSSteerVehicle's max speed.
 */
INLINE void OSSteerVehicle::set_max_speed(float maxSpeed)
{
	mVehicle->setMaxSpeed(maxSpeed);
}

/**
 * Returns the OSSteerVehicle's max speed.
 */
INLINE float OSSteerVehicle::get_max_speed() const
{
	return mVehicle->maxSpeed();
}

/**
 * Sets the OSSteerVehicle's forward.
 */
INLINE void OSSteerVehicle::set_forward(const LVector3f& forward)
{
	mVehicle->setForward(ossup::LVecBase3fToOpenSteerVec3(forward).normalize());
}

/**
 * Returns the OSSteerVehicle's forward.
 */
INLINE LVector3f OSSteerVehicle::get_forward() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(mVehicle->forward());
}

/**
 * Sets the OSSteerVehicle's side.
 */
INLINE void OSSteerVehicle::set_side(const LVector3f& side)
{
	mVehicle->setSide(ossup::LVecBase3fToOpenSteerVec3(side).normalize());
}

/**
 * Returns the OSSteerVehicle's side.
 */
INLINE LVector3f OSSteerVehicle::get_side() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(mVehicle->side());
}

/**
 * Sets the OSSteerVehicle's up.
 */
INLINE void OSSteerVehicle::set_up(const LVector3f& up)
{
	mVehicle->setUp(ossup::LVecBase3fToOpenSteerVec3(up).normalize());
}

/**
 * Returns the OSSteerVehicle's up.
 */
INLINE LVector3f OSSteerVehicle::get_up() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(mVehicle->up());
}

/**
 * Sets the OSSteerVehicle's position.
 */
INLINE void OSSteerVehicle::set_position(const LPoint3f& position)
{
	mVehicle->setPosition(ossup::LVecBase3fToOpenSteerVec3(position));
}

/**
 * Returns the OSSteerVehicle's position.
 */
INLINE LPoint3f OSSteerVehicle::get_position() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(mVehicle->position());
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
INLINE OSSteerVehicle::OSSteerVehicleMovType OSSteerVehicle::get_mov_type() const
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
	//serialization
	mSerializedDataTmpPtr = NULL;
	mPlayingTeam_ser = OSSteerPlugIn::NO_TEAM;
}

INLINE ostream &operator <<(ostream &out, const OSSteerVehicle& steerVehicle)
{
	steerVehicle.output(out);
	return out;
}

#endif /* OSSTEERVEHICLE_I_ */
