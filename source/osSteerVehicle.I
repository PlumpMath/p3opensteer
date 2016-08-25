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
 * Requires that OSSteerVehicle's up axis should be fixed along z.
 */
INLINE void OSSteerVehicle::set_up_axis_fixed(bool enable)
{
	mUpAxisFixed = enable;
}

/**
 * Returns if OSSteerVehicle's up axis is required to be fixed along z.
 */
INLINE bool OSSteerVehicle::get_up_axis_fixed() const
{
	return mUpAxisFixed;
}

/**
 * Sets the OSSteerVehicle's up axis fixed mode.
 * \note This mode establishes how the computation of the OSSteerVehicle's
 * orientation is carried out when the up axis is required to be fixed along z;
 * going from light to medium up to strong mode increase both precision and
 * computational cost.
 */
INLINE void OSSteerVehicle::set_up_axis_fixed_mode(
		OSSteerVehicleUpAxisFixedMode mode)
{
	mUpAxisFixedMode = mode;
}

/**
 * Returns the OSSteerVehicle's up axis fixed mode.
 */
INLINE OSSteerVehicle::OSSteerVehicleUpAxisFixedMode
		OSSteerVehicle::get_up_axis_fixed_mode() const
{
	return mUpAxisFixedMode;
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
 * Sets the OSSteerVehicle's start position.
 */
INLINE void OSSteerVehicle::set_start(const LPoint3f& position)
{
	static_cast<VehicleAddOn*>(mVehicle)->setStart(
			ossup::LVecBase3fToOpenSteerVec3(position));
}

/**
 * Returns the OSSteerVehicle's start position.
 */
INLINE LPoint3f OSSteerVehicle::get_start() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(
			static_cast<VehicleAddOn*>(mVehicle)->getStart());
}

/**
 * Sets the OSSteerVehicle's 'prediction time' steering parameter.
 */
INLINE void OSSteerVehicle::set_prediction_time(float predTime)
{
	static_cast<VehicleAddOn*>(mVehicle)->setPredictionTime(predTime);
}

/**
 * Returns the OSSteerVehicle's 'prediction time' steering parameter.
 */
INLINE float OSSteerVehicle::get_prediction_time() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getPredictionTime();
}

/**
 * Sets the OSSteerVehicle's 'minimum time to collision' steering parameter.
 */
INLINE void OSSteerVehicle::set_min_time_to_collision(float collTime)
{
	static_cast<VehicleAddOn*>(mVehicle)->setMinTimeToCollision(collTime);
}

/**
 * Returns the OSSteerVehicle's 'minimum time to collision' steering parameter.
 */
INLINE float OSSteerVehicle::get_min_time_to_collision() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getMinTimeToCollision();
}

/**
 * Sets the OSSteerVehicle's 'minimum separation distance' steering parameter.
 */
INLINE void OSSteerVehicle::set_min_separation_distance(float sepDist)
{
	static_cast<VehicleAddOn*>(mVehicle)->setMinSeparationDistance(sepDist);
}

/**
 * Returns the OSSteerVehicle's 'minimum separation distance' steering
 * parameter.
 */
INLINE float OSSteerVehicle::get_min_separation_distance() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getMinSeparationDistance();
}

/**
 * Sets the OSSteerVehicle's 'maximum distance' steering parameter.
 */
INLINE void OSSteerVehicle::set_max_distance(float dist)
{
	static_cast<VehicleAddOn*>(mVehicle)->setMaxDistance(dist);
}

/**
 * Returns the OSSteerVehicle's 'maximum distance' steering parameter.
 */
INLINE float OSSteerVehicle::get_max_distance() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getMaxDistance();
}

/**
 * Sets the OSSteerVehicle's cosine 'maximum angle' steering parameter.
 */
INLINE void OSSteerVehicle::set_cos_max_angle(float angle)
{
	static_cast<VehicleAddOn*>(mVehicle)->setCosMaxAngle(angle);
}

/**
 * Returns the OSSteerVehicle's 'cosine maximum angle' steering parameter.
 */
INLINE float OSSteerVehicle::get_cos_max_angle() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getCosMaxAngle();
}

/**
 * Sets the OSSteerVehicle's 'maximum prediction time' steering parameter.
 */
INLINE void OSSteerVehicle::set_max_prediction_time(float predTime)
{
	static_cast<VehicleAddOn*>(mVehicle)->setMaxPredictionTime(predTime);
}

/**
 * Returns the OSSteerVehicle's 'maximum prediction time' steering parameter.
 */
INLINE float OSSteerVehicle::get_max_prediction_time() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getMaxPredictionTime();
}
/**
 * Sets the OSSteerVehicle's 'target speed' steering parameter.
 */
INLINE void OSSteerVehicle::set_target_speed(float speed)
{
	static_cast<VehicleAddOn*>(mVehicle)->setTargetSpeed(speed);
}

/**
 * Returns the OSSteerVehicle's 'target speed' steering parameter.
 */
INLINE float OSSteerVehicle::get_target_speed() const
{
	return static_cast<VehicleAddOn*>(mVehicle)->getTargetSpeed();
}

/**
 * Returns if OSSteerVehicle is externally updated.
 */
INLINE bool OSSteerVehicle::get_external_update() const
{
	return mExternalUpdate;
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
 * Enables/disables a common OSSteerVehicle's event to be thrown.
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
	mUpAxisFixedMode = UP_AXIS_FIXED_LIGHT;
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
