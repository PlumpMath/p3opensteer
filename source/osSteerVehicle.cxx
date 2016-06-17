/**
 * \file osSteerVehicle.cpp
 *
 * \date 2016-05-13
 * \author consultit
 */
#include "osSteerVehicle.h"
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
 * Initializes the OSSteerVehicle with starting settings.
 */
void OSSteerVehicle::do_initialize()
{
	WPT(OSSteerManager)mTmpl = OSSteerManager::get_global_ptr();
	//set OSSteerVehicle parameters
	string param;
	float value;
	//external update
	mExternalUpdate = (mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("external_update")) ==
			string("true") ? true : false);
	//type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("type"));
	if (param == string("pedestrian"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Pedestrian<OSSteerVehicle> :
		mVehicle = new ossup::ExternalPedestrian<OSSteerVehicle>;
	}
	else if (param == string("boid"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Boid<OSSteerVehicle> :
		mVehicle = new ossup::ExternalBoid<OSSteerVehicle>;
	}
	else if (param == string("mp_wanderer"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MpWanderer<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMpWanderer<OSSteerVehicle>;
	}
	else if (param == string("mp_pursuer"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MpPursuer<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMpPursuer<OSSteerVehicle>;
	}
	else if (param == string("player"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Player<OSSteerVehicle> :
		mVehicle = new ossup::ExternalPlayer<OSSteerVehicle>;
	}
	else if (param == string("ball"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Ball<OSSteerVehicle> :
		mVehicle = new ossup::ExternalBall<OSSteerVehicle>;
	}
	else if (param == string("ctf_seeker"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::CtfSeeker<OSSteerVehicle> :
		mVehicle = new ossup::ExternalCtfSeeker<OSSteerVehicle>;
	}
	else if (param == string("ctf_enemy"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::CtfEnemy<OSSteerVehicle> :
		mVehicle = new ossup::ExternalCtfEnemy<OSSteerVehicle>;
	}
	else if (param == string("low_speed_turn"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::LowSpeedTurn<OSSteerVehicle> :
		mVehicle = new ossup::ExternalLowSpeedTurn<OSSteerVehicle>;
	}
	else if (param == string("map_driver"))
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
	}
	//register to SteerPlugIn objectId
	string mSteerPlugInObjectId =
	mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("add_to_plugin"));
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
	//get settings
	ossup::VehicleSettings settings;
	//mass
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("mass")).c_str(), NULL);
	settings.m_mass = (value >= 0.0 ? value : 1.0);
	//speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("speed")).c_str(),
			NULL);
	settings.m_speed = (value >= 0.0 ? value : -value);
	//max force
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("max_force")).c_str(),
			NULL);
	settings.m_maxForce = (value >= 0.0 ? value : -value);
	//max speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					string("max_speed")).c_str(),
			NULL);
	settings.m_maxSpeed = (value >= 0.0 ? value : 1.0);
	//set vehicle settings
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
	//thrown events
	mThrownEventsParam = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			string("thrown_events"));
	//
	//set entity and its related update method
	static_cast<VehicleAddOn*>(mVehicle)->setEntity(this);
	//
	! mExternalUpdate ?
	static_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
			&OSSteerVehicle::do_update_steer_vehicle) :
	static_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
			&OSSteerVehicle::do_external_update_steer_vehicle);

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
	//set the callbacks
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
	//clear all no more needed "Param" variables
	mThrownEventsParam.clear();
	//set vehicle's forward,( side,) up and position
	NodePath ownerNP/* = mOwnerObject->getNodePath()*/;
	ossup::VehicleSettings settings =
	static_cast<VehicleAddOn*>(mVehicle)->getSettings();
	settings.m_forward =
	ossup::LVecBase3fToOpenSteerVec3(
			ownerNP.get_parent().get_relative_vector(
					ownerNP, -LVector3f::forward())).normalize();
	settings.m_up = ossup::LVecBase3fToOpenSteerVec3(
			ownerNP.get_parent().get_relative_vector(
					ownerNP, LVector3f::up())).normalize();
	settings.m_position = ossup::LVecBase3fToOpenSteerVec3(
			ownerNP.get_pos());
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
	// set this NodePath
	NodePath thisNP = NodePath::any_path(this);
	// set the collide mask to avoid hit with the steer manager ray
	thisNP.set_collide_mask(~mTmpl->get_collide_mask() &
			thisNP.get_collide_mask());
	//add to SteerPlugIn, if requested
	PT(OSSteerPlugIn) plugIn = NULL;
	for (int index = 0;
			index < OSSteerManager::get_global_ptr()->get_num_steer_plug_ins();
			++index)
	{
		plugIn = DCAST(OSSteerPlugIn,
				OSSteerManager::get_global_ptr()->get_steer_plug_in(index).node());
		if (plugIn->get_name() == mSteerPlugInObjectId)
		{
			plugIn->addSteerVehicle(thisNP);
			break;
		}
	}
}

/**
 * On destruction cleanup.
 * Gives an OSSteerVehicle the ability to do any cleaning is necessary when
 * destroyed
 */
void OSSteerVehicle::do_finalize()
{
	//Remove from SteerPlugIn (if previously added)
	if (mSteerPlugIn)
	{
		mSteerPlugIn->removeSteerVehicle(NodePath::any_path(this));
	}
	//
	delete mVehicle;
	do_reset();
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
				mReferenceNP, updatedPos) + mHeigthCorrection;
		// get the collision height wrt the reference node path
		Pair<bool,float> gotCollisionZ = steerMgr->get_collision_height(pOrig,
				mReferenceNP);
		if (gotCollisionZ.get_first())
		{
			//updatedPos.z needs correction
			updatedPos.set_z(gotCollisionZ.get_second());
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
 */
void OSSteerVehicle::do_external_update_steer_vehicle(const float currentTime,
		const float elapsedTime)
{
	NodePath thisNP = NodePath::any_path(this);
	OpenSteer::Vec3 oldPos = mVehicle->position();
	//update vehicle's
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

	///Name of this RNCrowdAgent.
	dg.add_string(get_name());

	//XXX

	///The reference node path.
	manager->write_pointer(dg, mReferenceNP.node());
}

/**
 * Receives an array of pointers, one for each time manager->read_pointer()
 * was called in fillin(). Returns the number of pointers processed.
 */
int OSSteerVehicle::complete_pointers(TypedWritable **p_list, BamReader *manager)
{
	int pi = PandaNode::complete_pointers(p_list, manager);

	//XXX

	return pi;
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

	return node;
}

/**
 * This internal function is called by make_from_bam to read in all of the
 * relevant data from the BamFile for the new RNCrowdAgent.
 */
void OSSteerVehicle::fillin(DatagramIterator &scan, BamReader *manager)
{
	PandaNode::fillin(scan, manager);

	///Name of this OSSteerVehicle. string mName;
	set_name(scan.get_string());

	//XXX
}

//TypedObject semantics: hardcoded
TypeHandle OSSteerVehicle::_type_handle;
