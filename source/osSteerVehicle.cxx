/**
 * \file osSteerVehicle.cpp
 *
 * \date 2016-05-13
 * \author consultit
 */
#include "osSteerVehicle.h"
#include "throw_event.h"
#include "support/PlugIn_OneTurning.h"
#include "support/PlugIn_Pedestrian.h"
#include "support/PlugIn_Boids.h"
#include "support/PlugIn_MultiplePursuit.h"
#include "support/PlugIn_Soccer.h"
#include "support/PlugIn_CaptureTheFlag.h"
#include "support/PlugIn_LowSpeedTurn.h"
#include "support/PlugIn_MapDrive.h"

/**
 *
 */
OSSteerVehicle::OSSteerVehicle(const string& name) :
		PandaNode(name)
{
	mSteerPlugIn.clear();

	do_reset();
}

/**
 *
 */
OSSteerVehicle::~OSSteerVehicle()
{
}

/**
 * Sets the OSSteerVehicle type.
 * \note OSSteerVehicle's type can only be changed if it is not attached to any
 * OSSteerPlugIn(s).
 */
void OSSteerVehicle::set_vehicle_type(OSSteerVehicleType type)
{
	CONTINUE_IF_ELSE_V(!mSteerPlugIn)

	//save current OpenSteer vehicle's settings
	mVehicleSettings = get_settings();

	//create the new OpenSteer vehicle
	do_create_vehicle(type);

	//(re)set the new OpenSteer vehicle's settings
	set_settings(mVehicleSettings);
}

/**
 * Enables/disables OSSteerVehicle's external update.
 * Returns the value actually set.
 * \note OSSteerVehicle's external update can only be enabled/disabled if it
 * is not attached to any OSSteerPlugIn(s).
 */
bool OSSteerVehicle::enable_external_update(bool enable)
{
	CONTINUE_IF_ELSE_R(!mSteerPlugIn, mExternalUpdate)

	//set the external update
	mExternalUpdate = enable;
	//we need to re-create an OpenSteer vehicle with the same type
	set_vehicle_type(mVehicleType);
	//return the value set
	return mExternalUpdate;
}

/**
 * Creates actually the OpenSteer vehicle.
 */
void OSSteerVehicle::do_create_vehicle(OSSteerVehicleType type)
{
	//remove current steer vehicle if any
	if (mVehicle)
	{
		//delete the current steer vehicle
		delete mVehicle;
		mVehicle = NULL;
	}
	//create the steer vehicle
	mVehicleType = type;
	if ( mVehicleType== PEDESTRIAN)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Pedestrian<OSSteerVehicle> :
		mVehicle = new ossup::ExternalPedestrian<OSSteerVehicle>;
	}
	else if (mVehicleType == BOID)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Boid<OSSteerVehicle> :
		mVehicle = new ossup::ExternalBoid<OSSteerVehicle>;
	}
	else if (mVehicleType == MP_WANDERER)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MpWanderer<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMpWanderer<OSSteerVehicle>;
	}
	else if (mVehicleType == MP_PURSUER)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MpPursuer<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMpPursuer<OSSteerVehicle>;
	}
	else if (mVehicleType == PLAYER)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Player<OSSteerVehicle> :
		mVehicle = new ossup::ExternalPlayer<OSSteerVehicle>;
	}
	else if (mVehicleType == BALL)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Ball<OSSteerVehicle> :
		mVehicle = new ossup::ExternalBall<OSSteerVehicle>;
	}
	else if (mVehicleType == CTF_SEEKER)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::CtfSeeker<OSSteerVehicle> :
		mVehicle = new ossup::ExternalCtfSeeker<OSSteerVehicle>;
	}
	else if (mVehicleType == CTF_ENEMY)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::CtfEnemy<OSSteerVehicle> :
		mVehicle = new ossup::ExternalCtfEnemy<OSSteerVehicle>;
	}
	else if (mVehicleType == LOW_SPEED_TURN)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::LowSpeedTurn<OSSteerVehicle> :
		mVehicle = new ossup::ExternalLowSpeedTurn<OSSteerVehicle>;
	}
	else if (mVehicleType == MAP_DRIVER)
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MapDriver<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMapDriver<OSSteerVehicle>;
	}
	else
	{
		//default: one_turning
		! mExternalUpdate ?
		mVehicle = new ossup::OneTurning<OSSteerVehicle> :
		mVehicle = new ossup::ExternalOneTurning<OSSteerVehicle>;
		mVehicleType = ONE_TURNING;
	}
	//
	//set entity
	static_cast<VehicleAddOn*>(mVehicle)->setEntity(this);
	//set entity's update method
	! mExternalUpdate ?
	static_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
			&OSSteerVehicle::do_update_steer_vehicle) :
	static_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
			&OSSteerVehicle::do_external_update_steer_vehicle);
	//set callbacks
	//Path Following
	static_cast<VehicleAddOn*>(mVehicle)->setEntityPathFollowingMethod(
			&OSSteerVehicle::do_path_following);
	//Avoid Obstacle
	static_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidObstacleMethod(
			&OSSteerVehicle::do_avoid_obstacle);
	//Avoid Close Neighbor
	static_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidCloseNeighborMethod(
			&OSSteerVehicle::do_avoid_close_neighbor);
	//Avoid Neighbor
	static_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidNeighborMethod(
			&OSSteerVehicle::do_avoid_neighbor);
}

/**
 * Initializes the OSSteerVehicle with starting settings.
 * \note Internal use only.
 */
void OSSteerVehicle::do_initialize()
{
	WPT(OSSteerManager)mTmpl = OSSteerManager::get_global_ptr();
	//set OSSteerVehicle parameters
	string param;
	//external update
	mExternalUpdate = (mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("external_update")) ==
			string("true") ? true : false);
	//type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("vehicle_type"));
	//create the steer vehicle
	if (param == string("pedestrian"))
	{
		do_create_vehicle(PEDESTRIAN);
	}
	else if (param == string("boid"))
	{
		do_create_vehicle(BOID);
	}
	else if (param == string("mp_wanderer"))
	{
		do_create_vehicle(MP_WANDERER);
	}
	else if (param == string("mp_pursuer"))
	{
		do_create_vehicle(MP_PURSUER);
	}
	else if (param == string("player"))
	{
		do_create_vehicle(PLAYER);
	}
	else if (param == string("ball"))
	{
		do_create_vehicle(BALL);
	}
	else if (param == string("ctf_seeker"))
	{
		do_create_vehicle(CTF_SEEKER);
	}
	else if (param == string("ctf_enemy"))
	{
		do_create_vehicle(CTF_ENEMY);
	}
	else if (param == string("low_speed_turn"))
	{
		do_create_vehicle(LOW_SPEED_TURN);
	}
	else if (param == string("map_driver"))
	{
		do_create_vehicle(MAP_DRIVER);
	}
	else
	{
		do_create_vehicle(ONE_TURNING);
	}
	///Configure this OSSteerVehicle and the underlying OpenSteer vehicle
	float value;
	OSVehicleSettings settings;
	//get a NodePath for this
	NodePath thisNP = NodePath::any_path(this);
	//mov type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("mov_type"));
	if (param == string("kinematic"))
	{
		mMovType = OPENSTEER_KINEMATIC;
	}
	else
	{
		mMovType = OPENSTEER;
	}
	//up axis fixed
	mUpAxisFixed = (
			mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("up_axis_fixed")) ==
			string("true") ? true : false);
	//initialize settings with underlying OpenSteer vehicle's ones
	settings = static_cast<VehicleAddOn*>(mVehicle)->getSettings();
	//mass
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("mass")).c_str(), NULL);
	settings.set_mass(value >= 0.0 ? value : 1.0);
	//speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("speed")).c_str(),
			NULL);
	settings.set_speed(value >= 0.0 ? value : -value);
	//max force
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("max_force")).c_str(),
			NULL);
	settings.set_maxForce(value >= 0.0 ? value : -value);
	//max speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("max_speed")).c_str(),
			NULL);
	settings.set_maxSpeed(value >= 0.0 ? value : 1.0);
	//forward
	LVector3f forward = mReferenceNP.get_relative_vector(
			thisNP, -LVector3f::forward());
	settings.set_forward(forward);
	//up
	LVector3f up = mReferenceNP.get_relative_vector(
			thisNP, LVector3f::up());
	settings.set_up(up);
	//position
	settings.set_position(thisNP.get_pos());
	//set actually the OSSteerVehicle's and OpenSteer vehicle's settings
	set_settings(settings);
	//
	// set the collide mask to avoid hit with the steer manager ray
	thisNP.set_collide_mask(~mTmpl->get_collide_mask() &
			thisNP.get_collide_mask());
	//
	//thrown events
	string mThrownEventsParam = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("thrown_events"));
	//set thrown events if any
	unsigned int idx1, valueNum1;
	pvector<string> paramValuesStr1, paramValuesStr2;
	if (mThrownEventsParam != string(""))
	{
		//events specified
		//event1@[event_name1]@[frequency1][:...[:eventN@[event_nameN]@[frequencyN]]]
		paramValuesStr1 = parseCompoundString(mThrownEventsParam, ':');
		valueNum1 = paramValuesStr1.size();
		for (idx1 = 0; idx1 < valueNum1; ++idx1)
		{
			//eventX@[event_nameX]@[frequencyX]
			paramValuesStr2 = parseCompoundString(paramValuesStr1[idx1], '@');
			if (paramValuesStr2.size() >= 3)
			{
				OSEventThrown event;
				ThrowEventData eventData;
				//get default name prefix
				string objectType = get_name();
				//get name
				string name = paramValuesStr2[1];
				//get frequency
				float frequency = strtof(paramValuesStr2[2].c_str(), NULL);
				if (frequency <= 0.0)
				{
					frequency = 30.0;
				}
				//get event
				if (paramValuesStr2[0] == "move")
				{
					event = MOVEEVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_SteerVehicle_Move";
					}
				}
				else if (paramValuesStr2[0] == "steady")
				{
					event = STEADYEVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_SteerVehicle_Steady";
					}
				}
				else if (paramValuesStr2[0] == "path_following")
				{
					event = PATHFOLLOWINGEVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_SteerVehicle_PathFollowing";
					}
				}
				else if (paramValuesStr2[0] == "avoid_obstacle")
				{
					event = AVOIDOBSTACLEEVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_SteerVehicle_AvoidObstacle";
					}
				}
				else if (paramValuesStr2[0] == "avoid_close_neighbor")
				{
					event = AVOIDCLOSENEIGHBOREVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_SteerVehicle_AvoidCloseNeighbor";
					}
				}
				else if (paramValuesStr2[0] == "avoid_neighbor")
				{
					event = AVOIDNEIGHBOREVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_SteerVehicle_AvoidNeighbor";
					}
				}
				else
				{
					//paramValuesStr2[0] is not a suitable event:
					//continue with the next event
					continue;
				}
				//set event data
				eventData.mEnable = true;
				eventData.mEventName = name;
				eventData.mTimeElapsed = 0;
				eventData.mFrequency = frequency;
				//enable the event
				do_enable_steer_vehicle_event(event, eventData);
			}
		}
	}
	//
	//add to OSSteerPlugIn if requested
	string mSteerPlugInObjectId =
	mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("add_to_plugin"));
	PT(OSSteerPlugIn) plugIn = NULL;
	for (int index = 0;
			index < OSSteerManager::get_global_ptr()->get_num_steer_plug_ins();
			++index)
	{
		plugIn = DCAST(OSSteerPlugIn,
				OSSteerManager::get_global_ptr()->get_steer_plug_in(index).node());
		if (plugIn->get_name() == mSteerPlugInObjectId)
		{
			plugIn->add_steer_vehicle(thisNP);
			break;
		}
	}
}

/**
 * On destruction cleanup.
 * Gives an OSSteerVehicle the ability to do any cleaning is necessary when
 * destroyed.
 * \note Internal use only.
 */
void OSSteerVehicle::do_finalize()
{
	//Remove from SteerPlugIn (if previously added)
	if (mSteerPlugIn)
	{
		mSteerPlugIn->remove_steer_vehicle(NodePath::any_path(this));
	}
	//
	delete mVehicle;
	do_reset();
}

/**
 * Sets flock settings.
 * \note OSSteerVehicle should be not externally updated.
 * \note BOID OSSteerVehicle only.
 */
void OSSteerVehicle::set_flock_settings(const OSFlockSettings& settings)
{
	if (mVehicleType == BOID)
	{
		static_cast<ossup::Boid<OSSteerVehicle>*>(mVehicle)->setFlockParameters(
				settings.get_separation_radius(),
				settings.get_separation_angle(),
				settings.get_separation_weight(),
				settings.get_alignment_radius(),
				settings.get_alignment_angle(),
				settings.get_alignment_weight(),
				settings.get_cohesion_radius(),
				settings.get_cohesion_angle(),
				settings.get_cohesion_weight());
	}
}

/**
 * Returns flock settings.
 * Returns an OSFlockSettings with negative values on error.
 * \note BOID OSSteerVehicle only.
 */
OSFlockSettings OSSteerVehicle::get_flock_settings() const
{
	OSFlockSettings settings(OS_ERROR, OS_ERROR, OS_ERROR, OS_ERROR, OS_ERROR,
			OS_ERROR, OS_ERROR, OS_ERROR, OS_ERROR);
	if (mVehicleType == BOID)
	{
		static_cast<ossup::Boid<OSSteerVehicle>*>(mVehicle)->getFlockParameters(
				settings.separation_radius(),
				settings.separation_angle(),
				settings.separation_weight(),
				settings.alignment_radius(),
				settings.alignment_angle(),
				settings.alignment_weight(),
				settings.cohesion_radius(),
				settings.cohesion_angle(),
				settings.cohesion_weight());
	}
	return settings;
}

/**
 * Returns the OSSteerVehicle's current playing team, or a negative value on
 * error.
 * \note The team can only be changed through OSSteerPlugIn API.
 * \note SOCCER OSSteerVehicle only.
 */
OSSteerPlugIn::OSPlayingTeam OSSteerVehicle::get_playing_team() const
{
	if (mVehicleType == PLAYER)
	{
		if (static_cast<ossup::Player<OSSteerVehicle>*>(mVehicle)->m_TeamAssigned)
		{
			OSSteerPlugIn::OSPlayingTeam team =
					(static_cast<ossup::Player<OSSteerVehicle>*>(mVehicle)->b_ImTeamA ?
							OSSteerPlugIn::TEAM_A : OSSteerPlugIn::TEAM_B);
			nassertr_always(mPlayingTeam_ser == team,
					(OSSteerPlugIn::OSPlayingTeam)OS_ERROR)

			return team;
		}
		else
		{
			nassertr_always(mPlayingTeam_ser == OSSteerPlugIn::NO_TEAM,
					(OSSteerPlugIn::OSPlayingTeam)OS_ERROR)

			return OSSteerPlugIn::NO_TEAM;
		}
	}
	return (OSSteerPlugIn::OSPlayingTeam) OS_ERROR;
}

/**
 * Sets steering speed.
 * \note OSSteerVehicle should be not externally updated.
 * \note LOW_SPEED_TURN OSSteerVehicle only.
 */
void OSSteerVehicle::set_steering_speed(float steeringSpeed)
{
	if (mVehicleType == LOW_SPEED_TURN)
	{
		static_cast<ossup::LowSpeedTurn<OSSteerVehicle>*>(mVehicle)->steeringSpeed =
				steeringSpeed;
	}
}

/**
 * Returns steering speed.
 * Returns a negative value on error.
 * \note LOW_SPEED_TURN OSSteerVehicle only.
 */
float OSSteerVehicle::get_steering_speed() const
{
	return mVehicleType == LOW_SPEED_TURN ?
			static_cast<ossup::LowSpeedTurn<OSSteerVehicle>*>(mVehicle)->steeringSpeed :
			OS_ERROR;
}

/**
 * Enables/disables OSSteerVehicle to reverse direction when it reaches a
 * pathway end-point (default: false).
 * \note PEDESTRIAN OSSteerVehicle only.
 */
void OSSteerVehicle::set_reverse_at_end_point(bool enable)
{
	if (mVehicleType == PEDESTRIAN)
	{
		static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle)->useDirectedPathFollowing =
				enable;
	}
}

/**
 * Returns if OSSteerVehicle reverses direction when it reaches a pathway
 * end-point, or a negative value on error.
 * \note PEDESTRIAN OSSteerVehicle only.
 */
bool OSSteerVehicle::get_reverse_at_end_point() const
{
	return (mVehicleType == PEDESTRIAN) ?
			static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle)->useDirectedPathFollowing :
			OS_ERROR;
}

/**
 * Enables/disables OSSteerVehicle's wander behavior (default: false);
 * \note PEDESTRIAN OSSteerVehicle only.
 */
void OSSteerVehicle::set_wander_behavior(bool enable)
{
	if (mVehicleType == PEDESTRIAN)
	{
		static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle)->wanderSwitch =
				enable;
	}
}

/**
 * Returns if OSSteerVehicle has wander behavior, or a negative value on error.
 * \note PEDESTRIAN OSSteerVehicle only.
 */
bool OSSteerVehicle::get_wander_behavior() const
{
	return (mVehicleType == PEDESTRIAN) ?
			static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle)->wanderSwitch :
			OS_ERROR;
}

/**
 * Sets first/second pathway end points (default: first/last specified points in
 * the pathway of the OSSteerPlugIn).
 * \note PEDESTRIAN OSSteerVehicle only.
 */
void OSSteerVehicle::set_pathway_end_points(const ValueList<LPoint3f>& points)
{
	if (mVehicleType == PEDESTRIAN)
	{
		CONTINUE_IF_ELSE_V(points.size() >= 2)

		static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle)->pathEndpoint0 =
				ossup::LVecBase3fToOpenSteerVec3(points[0]);
		static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle)->pathEndpoint1 =
				ossup::LVecBase3fToOpenSteerVec3(points[1]);
	}
}

/**
 * Returns first/second pathway end points, or empty list on error.
 * \note PEDESTRIAN OSSteerVehicle only.
 */
ValueList<LPoint3f> OSSteerVehicle::get_pathway_end_points() const
{
	ValueList<LPoint3f> points;
	if (mVehicleType == PEDESTRIAN)
	{
		ossup::Pedestrian<OSSteerVehicle>* vehicle =
				static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle);
		points.add_value(
				ossup::OpenSteerVec3ToLVecBase3f(vehicle->pathEndpoint0));
		points.add_value(
				ossup::OpenSteerVec3ToLVecBase3f(vehicle->pathEndpoint1));
	}
	return points;
}

/**
 * Sets OSSteerVehicle's direction for pathway following:
 * - UPSTREAM
 * - DOWNSTREAM
 * By default direction is chosen randomly at creation time.
 * \note PEDESTRIAN OSSteerVehicle only.
 */
void OSSteerVehicle::set_pathway_direction(OSPathDirection direction)
{
	if (mVehicleType == PEDESTRIAN)
	{
		ossup::Pedestrian<OSSteerVehicle>* vehicle =
				static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle);
		switch (direction)
		{
		case UPSTREAM:
			vehicle->pathDirection = 1;
			break;
		case DOWNSTREAM:
			vehicle->pathDirection = -1;
			break;
		default:
			break;
		}
	}
}

/**
 * Returns OSSteerVehicle's direction for pathway following, or a negative
 * value on error.
 * \note PEDESTRIAN OSSteerVehicle only.
 */
OSSteerVehicle::OSPathDirection OSSteerVehicle::get_pathway_direction() const
{
	OSPathDirection result = (OSPathDirection) OS_ERROR;
	if (mVehicleType == PEDESTRIAN)
	{
		ossup::Pedestrian<OSSteerVehicle>* vehicle =
				static_cast<ossup::Pedestrian<OSSteerVehicle>*>(mVehicle);
		switch (vehicle->pathDirection)
		{
		case 1:
			result = UPSTREAM;
			break;
		case -1:
			result = DOWNSTREAM;
			break;
		default:
			break;
		}
	}
	return result;
}

/**
 * Writes a sensible description of the OSSteerVehicle to the indicated output
 * stream.
 */
void OSSteerVehicle::output(ostream &out) const
{
	out << get_type() << " " << get_name();
}

/**
 * Updates the OSSteerVehicle.
 * Called by the underlying OpenSteer component update.
 * \note Internal use only.
 */
void OSSteerVehicle::do_update_steer_vehicle(const float currentTime,
		const float elapsedTime)
{
	NodePath thisNP = NodePath::any_path(this);
	LPoint3f updatedPos = ossup::OpenSteerVec3ToLVecBase3f(
			mVehicle->position());
	//update node path position
	if ((mMovType == OPENSTEER_KINEMATIC) && (mVehicle->speed() > 0.0))
	{
		// get steer manager
		WPT(OSSteerManager)steerMgr = OSSteerManager::get_global_ptr();
		// correct panda's Z: set the collision ray origin wrt collision root
		LPoint3f pOrig = steerMgr->get_collision_root().get_relative_point(
				mReferenceNP, updatedPos) + mHeigthCorrection * 2.0;
		// get the collision height wrt the reference node path
		Pair<bool,float> gotCollisionZ = steerMgr->get_collision_height(pOrig,
				mReferenceNP);
		if (gotCollisionZ.first())
		{
			//updatedPos.z needs correction
			updatedPos.set_z(gotCollisionZ.second());
			//correct vehicle position
			mVehicle->setPosition(ossup::LVecBase3fToOpenSteerVec3(updatedPos));
		}
	}
	thisNP.set_pos(updatedPos);

	if (mVehicle->speed() > 0.0)
	{
		//update node path dir
		mUpAxisFixed ?
		//up axis fixed: z
				thisNP.heads_up(
						updatedPos
								- ossup::OpenSteerVec3ToLVecBase3f(
										mVehicle->forward()), LVector3f::up()) :
				//up axis free: from mVehicle
				thisNP.heads_up(
						updatedPos
								- ossup::OpenSteerVec3ToLVecBase3f(
										mVehicle->forward()),
						ossup::OpenSteerVec3ToLVecBase3f(mVehicle->up()));

		//handle Move/Steady events
		//throw Move event (if enabled)
		if (mMove.mEnable)
		{
			do_throw_event(mMove);
		}
		//reset Steady event (if enabled and if thrown)
		if (mSteady.mEnable && mSteady.mThrown)
		{
			mSteady.mThrown = false;
			mSteady.mTimeElapsed = 0.0;
		}
	}
	else //mVehicle->speed() == 0.0
	{
		//handle Move/Steady events
		//mVehicle.speed == 0.0
		//reset Move event (if enabled and if thrown)
		if (mMove.mEnable && mMove.mThrown)
		{
			mMove.mThrown = false;
			mMove.mTimeElapsed = 0.0;
		}
		//throw Steady event (if enabled)
		if (mSteady.mEnable)
		{
			do_throw_event(mSteady);
		}
	}

	//handle SteerLibrary events
	do_handle_steer_library_event(mPathFollowing, mPFCallbackCalled);
	do_handle_steer_library_event(mAvoidObstacle, mAOCallbackCalled);
	do_handle_steer_library_event(mAvoidCloseNeighbor, mACNCallbackCalled);
	do_handle_steer_library_event(mAvoidNeighbor, mANCallbackCalled);
}

/**
 * Updates the OSSteerVehicle.
 * Called when component is updated outside of OpenSteer.
 * \note Internal use only.
 */
void OSSteerVehicle::do_external_update_steer_vehicle(const float currentTime,
		const float elapsedTime)
{
	NodePath thisNP = NodePath::any_path(this);
	OpenSteer::Vec3 oldPos = mVehicle->position();
	//update steer vehicle's
	//position,
	mVehicle->setPosition(
			ossup::LVecBase3fToOpenSteerVec3(
					thisNP.get_pos() - mHeigthCorrection));
	//forward,
	mVehicle->setForward(
			ossup::LVecBase3fToOpenSteerVec3(
					mReferenceNP.get_relative_vector(thisNP,
							-LVector3f::forward())).normalize());
	//up,
	mVehicle->setUp(
			ossup::LVecBase3fToOpenSteerVec3(
					mReferenceNP.get_relative_vector(thisNP, LVector3f::up())).normalize());
	//side,
	mVehicle->setUnitSideFromForwardAndUp();
	//speed (elapsedTime should be != 0)
	mVehicle->setSpeed((mVehicle->position() - oldPos).length() / elapsedTime);
	//
	//no event thrown: external updating sub-system will do, if expected
}

/**
 * Enables/disables event throwing.
 * \note Internal use only.
 */
void OSSteerVehicle::do_enable_steer_vehicle_event(OSEventThrown event,
		ThrowEventData eventData)
{
	//some checks
	nassertv_always(!eventData.mEventName.empty())

	if (eventData.mFrequency <= 0.0)
	{
		eventData.mFrequency = 30.0;
	}

	switch (event)
	{
	case MOVEEVENT:
		if (mMove.mEnable != eventData.mEnable)
		{
			mMove = eventData;
			mMove.mTimeElapsed = 0;
		}
		break;
	case STEADYEVENT:
		if (mSteady.mEnable != eventData.mEnable)
		{
			mSteady = eventData;
			mSteady.mTimeElapsed = 0;
		}
		break;
	case PATHFOLLOWINGEVENT:
		if (mPathFollowing.mEnable != eventData.mEnable)
		{
			mPathFollowing = eventData;
			mPathFollowing.mTimeElapsed = 0;
			mPFCallbackCalled = false;
		}
		break;
	case AVOIDOBSTACLEEVENT:
		if (mAvoidObstacle.mEnable != eventData.mEnable)
		{
			mAvoidObstacle = eventData;
			mAvoidObstacle.mTimeElapsed = 0;
			mAOCallbackCalled = false;
		}
		break;
	case AVOIDCLOSENEIGHBOREVENT:
		if (mAvoidCloseNeighbor.mEnable != eventData.mEnable)
		{
			mAvoidCloseNeighbor = eventData;
			mAvoidCloseNeighbor.mTimeElapsed = 0;
			mACNCallbackCalled = false;
		}
		break;
	case AVOIDNEIGHBOREVENT:
		if (mAvoidNeighbor.mEnable != eventData.mEnable)
		{
			mAvoidNeighbor = eventData;
			mAvoidNeighbor.mTimeElapsed = 0;
			mANCallbackCalled = false;
		}
		break;
	default:
		break;
	}
}

/**
 * Path following callback.
 * \note Internal use only.
 */
void OSSteerVehicle::do_path_following(const OpenSteer::Vec3& future,
		const OpenSteer::Vec3& onPath, const OpenSteer::Vec3& target,
		const float outside)
{
	//handle Path Following event
	if (mPathFollowing.mEnable)
	{
		do_throw_event(mPathFollowing);
		//set the flag
		mPFCallbackCalled = true;
	}
}

/**
 * Avoid obstacle callback.
 * \note Internal use only.
 */
void OSSteerVehicle::do_avoid_obstacle(const float minDistanceToCollision)
{
	//handle Avoid Obstacle event
	if (mAvoidObstacle.mEnable)
	{
		do_throw_event(mAvoidObstacle);
		//set the flag
		mAOCallbackCalled = true;
	}
}

/**
 * Avoid close neighbor callback.
 * \note Internal use only.
 */
void OSSteerVehicle::do_avoid_close_neighbor(
		const OpenSteer::AbstractVehicle& other, const float additionalDistance)
{
	//handle Avoid Close Neighbor event
	if (mAvoidCloseNeighbor.mEnable)
	{
		do_throw_event(mAvoidCloseNeighbor);
		//set the flag
		mACNCallbackCalled = true;
	}
}

/**
 * Avoid neighbor callback.
 * \note Internal use only.
 */
void OSSteerVehicle::do_avoid_neighbor(const OpenSteer::AbstractVehicle& threat,
		const float steer, const OpenSteer::Vec3& ourFuture,
		const OpenSteer::Vec3& threatFuture)
{
	//handle Avoid Neighbor event
	if (mAvoidNeighbor.mEnable)
	{
		do_throw_event(mAvoidNeighbor);
		//set the flag
		mANCallbackCalled = true;
	}
}

/**
 * Throws an event when needed.
 * \note Internal use only.
 */
void OSSteerVehicle::do_throw_event(ThrowEventData& eventData)
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

/**
 * Handles a OpenSteer library event.
 * \note Internal use only.
 */
void OSSteerVehicle::do_handle_steer_library_event(ThrowEventData& eventData,
		bool callbackCalled)
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

//TypedWritable API
/**
 * Tells the BamReader how to create objects of type OSSteerVehicle.
 */
void OSSteerVehicle::register_with_read_factory()
{
	BamReader::get_factory()->register_factory(get_class_type(), make_from_bam);
}

/**
 * Writes the contents of this object to the datagram for shipping out to a
 * Bam file.
 */
void OSSteerVehicle::write_datagram(BamWriter *manager, Datagram &dg)
{
	PandaNode::write_datagram(manager, dg);

	///Name of this OSSteerVehicle.
	dg.add_string(get_name());

	///The type of this OSSteerPlugIn.
	dg.add_uint8((uint8_t) mVehicleType);

	///The movement type of this OSSteerPlugIn.
	dg.add_uint8((uint8_t) mMovType);

	///OSSteerVehicle settings.
	mVehicleSettings = get_settings();
	mVehicleSettings.write_datagram(dg);

	///Height correction for kinematic OSSteerVehicle(s).
	mHeigthCorrection.write_datagram(dg);

	///Flag for up axis fixed (z).
	dg.add_bool(mUpAxisFixed);

	///External update.
	dg.add_bool(mExternalUpdate);

	/**
	 * \name Throwing OSSteerVehicle events.
	 */
	///@{
	dg.add_bool(mPFCallbackCalled);
	dg.add_bool(mAOCallbackCalled);
	dg.add_bool(mACNCallbackCalled);
	dg.add_bool(mANCallbackCalled);
	mMove.write_datagram(dg);
	mSteady.write_datagram(dg);
	mPathFollowing.write_datagram(dg);
	mAvoidObstacle.write_datagram(dg);
	mAvoidCloseNeighbor.write_datagram(dg);
	mAvoidNeighbor.write_datagram(dg);
	///@}

	///The OSSteerPlugIn this OSSteerVehicle is added to.
	manager->write_pointer(dg, mSteerPlugIn);

	///The reference node path.
	manager->write_pointer(dg, mReferenceNP.node());

	///SPECIFICS
	if(mVehicleType == ONE_TURNING)
	{
		/*do nothing*/;
	}
	if(mVehicleType == PEDESTRIAN)
	{
		dg.add_bool(get_reverse_at_end_point());
		dg.add_bool(get_wander_behavior());
		ValueList<LPoint3f> points = get_pathway_end_points();
		dg.add_uint32(points.size());
		for (int i = 0; i != points.size(); ++i)
		{
			points[i].write_datagram(dg);
		}
		dg.add_uint8((uint8_t) get_pathway_direction());
	}
	if(mVehicleType == BOID)
	{
		get_flock_settings().write_datagram(dg);
	}
	if(mVehicleType == MP_WANDERER)
	{
		/*do nothing*/;
	}
	if(mVehicleType == MP_PURSUER)
	{
		/*do nothing*/;
	}
	if(mVehicleType == PLAYER)
	{
		dg.add_uint8((uint8_t) get_playing_team());
	}
	if(mVehicleType == BALL)
	{
		/*do nothing*/;
	}
	if(mVehicleType == CTF_SEEKER)
	{
		;
	}
	if(mVehicleType == CTF_ENEMY)
	{
		;
	}
	if(mVehicleType == LOW_SPEED_TURN)
	{
		dg.add_stdfloat(get_steering_speed());
	}
	if(mVehicleType == MAP_DRIVER)
	{
		;
	}
}

/**
 * Receives an array of pointers, one for each time manager->read_pointer()
 * was called in fillin(). Returns the number of pointers processed.
 */
int OSSteerVehicle::complete_pointers(TypedWritable **p_list, BamReader *manager)
{
	int pi = PandaNode::complete_pointers(p_list, manager);

	///The OSSteerPlugIn this OSSteerVehicle is added to.
	mSteerPlugIn = DCAST(OSSteerPlugIn, p_list[pi++]);

	///The reference node path.
	PT(PandaNode)referenceNPPandaNode = DCAST(PandaNode, p_list[pi++]);
	mReferenceNP = NodePath::any_path(referenceNPPandaNode);

	return pi;
}

/**
 * Called by the BamReader to perform any final actions needed for setting up
 * the object after all objects have been read and all pointers have been
 * completed.
 */
void OSSteerVehicle::finalize(BamReader *manager)
{
	//1: remove the old OpenSteer vehicle from real update list (if needed)
	if (mSteerPlugIn)
	{
		static_cast<ossup::PlugIn*>(&mSteerPlugIn->get_abstract_plug_in())->removeVehicle(
				mVehicle);
	}
	//2: (re)set type
	//create the new OpenSteer vehicle
	do_create_vehicle(mVehicleType);
	//set the new OpenSteer vehicle's settings
	set_settings(mVehicleSettings);
	//3: add the new OpenSteer vehicle to real update list (if needed), by
	//checking if plug-in has gained its final type (i.e. finalized)
	if (mSteerPlugIn
			&& (mSteerPlugIn->check_steer_vehicle_compatibility(
					NodePath::any_path(this))))
	{
		static_cast<ossup::PlugIn*>(&mSteerPlugIn->get_abstract_plug_in())->addVehicle(
				mVehicle);
	}

	///SPECIFICS
	if(mVehicleType == ONE_TURNING)
	{
		/*do nothing*/;
	}
	if(mVehicleType == PEDESTRIAN)
	{
		set_reverse_at_end_point(mReverseAtEndPoint_ser);
		set_wander_behavior(mWanderBehavior_ser);
		set_pathway_end_points(mPathwayEndPoints_ser);
		mPathwayEndPoints_ser.clear();
		set_pathway_direction(mPathwayDirection_ser);
	}
	if(mVehicleType == BOID)
	{
		set_flock_settings(mFlockSettings_ser);
	}

	if(mVehicleType == MP_WANDERER)
	{
		/*do nothing*/;
	}
	if(mVehicleType == MP_PURSUER)
	{
		/*do nothing*/;
	}
	if(mVehicleType == PLAYER)
	{
		// check if plug-in has gained its final type (i.e. finalized)
		if (mSteerPlugIn
				&& dynamic_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(
						&mSteerPlugIn->get_abstract_plug_in()))
		{
			mSteerPlugIn->add_player_to_team(this, mPlayingTeam_ser);
		}
	}
	if(mVehicleType == BALL)
	{
		/*do nothing*/;
	}
	if(mVehicleType == CTF_SEEKER)
	{
		;
	}
	if(mVehicleType == CTF_ENEMY)
	{
		;
	}
	if(mVehicleType == LOW_SPEED_TURN)
	{
		set_steering_speed(mSteeringSpeed_ser);
	}
	if(mVehicleType == MAP_DRIVER)
	{
		;
	}
}

/**
 * This function is called by the BamReader's factory when a new object of
 * type OSSteerVehicle is encountered in the Bam file.  It should create the
 * OSSteerVehicle and extract its information from the file.
 */
TypedWritable *OSSteerVehicle::make_from_bam(const FactoryParams &params)
{
	// return NULL if OSSteerManager if doesn't exist
	nassertr_always(OSSteerManager::get_global_ptr(), NULL)

	// create a OSSteerVehicle with default parameters' values: they'll be restored later
	OSSteerManager::get_global_ptr()->set_parameters_defaults(
			OSSteerManager::STEERVEHICLE);
	OSSteerVehicle *node = DCAST(OSSteerVehicle,
			OSSteerManager::get_global_ptr()->create_steer_vehicle(
					"SteerVehicle").node());

	DatagramIterator scan;
	BamReader *manager;

	parse_params(params, scan, manager);
	node->fillin(scan, manager);
	manager->register_finalize(node);

	return node;
}

/**
 * This internal function is called by make_from_bam to read in all of the
 * relevant data from the BamFile for the new OSSteerVehicle.
 */
void OSSteerVehicle::fillin(DatagramIterator &scan, BamReader *manager)
{
	PandaNode::fillin(scan, manager);

	///Name of this OSSteerVehicle.
	set_name(scan.get_string());

	///The type of this OSSteerPlugIn.
	mVehicleType = (OSSteerVehicleType)scan.get_uint8();

	///The movement type of this OSSteerPlugIn.
	mMovType = (OSSteerVehicleMovType)scan.get_uint8();

	///OSSteerVehicle settings.
	mVehicleSettings.read_datagram(scan);

	///Height correction for kinematic OSSteerVehicle(s).
	mHeigthCorrection.read_datagram(scan);

	///Flag for up axis fixed (z).
	mUpAxisFixed = scan.get_bool();

	///External update.
	mExternalUpdate = scan.get_bool();

	/**
	 * \name Throwing OSSteerVehicle events.
	 */
	///@{
	mPFCallbackCalled = scan.get_bool();
	mAOCallbackCalled = scan.get_bool();
	mACNCallbackCalled = scan.get_bool();
	mANCallbackCalled = scan.get_bool();
	mMove.read_datagram(scan);
	mSteady.read_datagram(scan);
	mPathFollowing.read_datagram(scan);
	mAvoidObstacle.read_datagram(scan);
	mAvoidCloseNeighbor.read_datagram(scan);
	mAvoidNeighbor.read_datagram(scan);
	///@}

	///The OSSteerPlugIn this OSSteerVehicle is added to.
	manager->read_pointer(scan);

	///The reference node path.
	manager->read_pointer(scan);

	///SPECIFICS
	if(mVehicleType == ONE_TURNING)
	{
		/*do nothing*/;
	}
	if(mVehicleType == PEDESTRIAN)
	{
		mReverseAtEndPoint_ser = scan.get_bool();
		mWanderBehavior_ser = scan.get_bool();
		mPathwayEndPoints_ser.clear();
		unsigned int sizeP = scan.get_uint32();
		for (unsigned int i = 0; i < sizeP; ++i)
		{
			LPoint3f point;
			point.read_datagram(scan);
			mPathwayEndPoints_ser.add_value(point);
		}
		mPathwayDirection_ser = (OSPathDirection) scan.get_uint8();
	}
	if(mVehicleType == BOID)
	{
		mFlockSettings_ser.read_datagram(scan);
	}
	if(mVehicleType == MP_WANDERER)
	{
		/*do nothing*/;
	}
	if(mVehicleType == MP_PURSUER)
	{
		/*do nothing*/;
	}
	if(mVehicleType == PLAYER)
	{
		mPlayingTeam_ser = (OSSteerPlugIn::OSPlayingTeam)scan.get_uint8();
	}
	if(mVehicleType == BALL)
	{
		/*do nothing*/;
	}
	if(mVehicleType == CTF_SEEKER)
	{
		;
	}
	if(mVehicleType == CTF_ENEMY)
	{
		;
	}
	if(mVehicleType == LOW_SPEED_TURN)
	{
		mSteeringSpeed_ser = scan.get_stdfloat();
	}
	if(mVehicleType == MAP_DRIVER)
	{
		;
	}
}

//TypedObject semantics: hardcoded
TypeHandle OSSteerVehicle::_type_handle;
