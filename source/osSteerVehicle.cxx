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

OSSteerVehicle::OSSteerVehicle(const string& name) :
		PandaNode(name)
{
	mSteerPlugIn.clear();

	do_reset();
}

OSSteerVehicle::~OSSteerVehicle()
{
}

void OSSteerVehicle::do_initialize()
{
	WPT(OSSteerManager)mTmpl = OSSteerManager::get_global_ptr();
	//set OSSteerVehicle parameters
	std::string param;
	float value;
	//external update
	mExternalUpdate = (mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					std::string("external_update")) ==
			std::string("true") ? true : false);
	//type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			std::string("type"));
	if (param == std::string("pedestrian"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Pedestrian<OSSteerVehicle> :
		mVehicle = new ossup::ExternalPedestrian<OSSteerVehicle>;
	}
	else if (param == std::string("boid"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Boid<OSSteerVehicle> :
		mVehicle = new ossup::ExternalBoid<OSSteerVehicle>;
	}
	else if (param == std::string("mp_wanderer"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MpWanderer<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMpWanderer<OSSteerVehicle>;
	}
	else if (param == std::string("mp_pursuer"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::MpPursuer<OSSteerVehicle> :
		mVehicle = new ossup::ExternalMpPursuer<OSSteerVehicle>;
	}
	else if (param == std::string("player"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Player<OSSteerVehicle> :
		mVehicle = new ossup::ExternalPlayer<OSSteerVehicle>;
	}
	else if (param == std::string("ball"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::Ball<OSSteerVehicle> :
		mVehicle = new ossup::ExternalBall<OSSteerVehicle>;
	}
	else if (param == std::string("ctf_seeker"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::CtfSeeker<OSSteerVehicle> :
		mVehicle = new ossup::ExternalCtfSeeker<OSSteerVehicle>;
	}
	else if (param == std::string("ctf_enemy"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::CtfEnemy<OSSteerVehicle> :
		mVehicle = new ossup::ExternalCtfEnemy<OSSteerVehicle>;
	}
	else if (param == std::string("low_speed_turn"))
	{
		! mExternalUpdate ?
		mVehicle = new ossup::LowSpeedTurn<OSSteerVehicle> :
		mVehicle = new ossup::ExternalLowSpeedTurn<OSSteerVehicle>;
	}
	else if (param == std::string("map_driver"))
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
			std::string("add_to_plugin"));
	//mov type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			std::string("mov_type"));
	if (param == std::string("kinematic"))
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
					std::string("up_axis_fixed")) ==
							std::string("true") ? true : false);
	//get settings
	ossup::VehicleSettings settings;
	//mass
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					std::string("mass")).c_str(), NULL);
	settings.m_mass = (value >= 0.0 ? value : 1.0);
	//radius
	mInputRadius = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					std::string("radius")).c_str(),
			NULL);
	//speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					std::string("speed")).c_str(),
			NULL);
	settings.m_speed = (value >= 0.0 ? value : -value);
	//max force
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					std::string("max_force")).c_str(),
			NULL);
	settings.m_maxForce = (value >= 0.0 ? value : -value);
	//max speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
					std::string("max_speed")).c_str(),
			NULL);
	settings.m_maxSpeed = (value >= 0.0 ? value : 1.0);
//	//ray mask XXX
//	param = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE, std::string("ray_mask"));
//	if (param == std::string("all_on"))
//	{
//		mRayMask = BitMask32::all_on();
//	}
//	else if (param == std::string("all_off"))
//	{
//		mRayMask = BitMask32::all_off();
//	}
//	else
//	{
//		uint32_t mask = (uint32_t) strtol(param.c_str(), NULL, 0);
//		mRayMask.set_word(mask);
//	}
	//set vehicle settings
	static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
	//thrown events
	mThrownEventsParam = mTmpl->get_parameter_value(OSSteerManager::STEERVEHICLE,
			std::string("thrown_events"));
	//
	LVecBase3f modelDims;
	LVector3f modelDeltaCenter;
	float modelRadius;
//	GamePhysicsManager::GetSingletonPtr()->getBoundingDimensions( XXX
//			mOwnerObject->getNodePath(), modelDims, modelDeltaCenter,
//			modelRadius);
	//set definitive radius
	if (mInputRadius <= 0.0)
	{
		// store new radius into settings
		ossup::VehicleSettings settings =
		static_cast<VehicleAddOn*>(mVehicle)->getSettings();
		settings.m_radius = modelRadius;
		static_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
	}
//	//set physics parameters XXX
//	mMaxError = modelDims.get_z();
//	mDeltaRayOrig = LVector3f(0, 0, mMaxError);
//	mDeltaRayDown = LVector3f(0, 0, -10 * mMaxError);
//	//correct height if there is a Physics or PhysicsControl component
//	//for raycast into update
//	if (mOwnerObject->getComponent(ComponentFamilyType("Physics"))
//			or mOwnerObject->getComponent(
//					ComponentFamilyType("PhysicsControl")))
//	{
//		mCorrectHeightRigidBody = modelDims.get_z() / 2.0;
//	}
//	else
//	{
//		mCorrectHeightRigidBody = 0.0;
//	}
	//set entity and its related update method
	static_cast<VehicleAddOn*>(mVehicle)->setEntity(this);
	//
	! mExternalUpdate ?
	static_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
			&OSSteerVehicle::doUpdateSteerVehicle) :
	static_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
			&OSSteerVehicle::doExternalUpdateSteerVehicle);

//	//set the bullet physics XXX
//	mBulletWorld = GamePhysicsManager::GetSingletonPtr()->bulletWorld();

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
				EventThrown event;
				ThrowEventData eventData;
				//get default name prefix
//				std::string objectType = std::string( XXX
//						mOwnerObject->objectTmpl()->objectType());
				//get name
				std::string name = paramValuesStr2[1];
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
//						name = objectType + "_SteerVehicle_Move"; XXX
					}
				}
				else if (paramValuesStr2[0] == "steady")
				{
					event = STEADYEVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_Steady"; XXX
					}
				}
				else if (paramValuesStr2[0] == "path_following")
				{
					event = PATHFOLLOWINGEVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_PathFollowing"; XXX
					}
				}
				else if (paramValuesStr2[0] == "avoid_obstacle")
				{
					event = AVOIDOBSTACLEEVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_AvoidObstacle"; XXX
					}
				}
				else if (paramValuesStr2[0] == "avoid_close_neighbor")
				{
					event = AVOIDCLOSENEIGHBOREVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_AvoidCloseNeighbor"; XXX
					}
				}
				else if (paramValuesStr2[0] == "avoid_neighbor")
				{
					event = AVOIDNEIGHBOREVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_AvoidNeighbor"; XXX
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
				doEnableSteerVehicleEvent(event, eventData);
			}
		}
	}
	//set the callbacks
	//Path Following
	static_cast<VehicleAddOn*>(mVehicle)->setEntityPathFollowingMethod(
			&OSSteerVehicle::doPathFollowing);
	//Avoid Obstacle
	static_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidObstacleMethod(
			&OSSteerVehicle::doAvoidObstacle);
	//Avoid Close Neighbor
	static_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidCloseNeighborMethod(
			&OSSteerVehicle::doAvoidCloseNeighbor);
	//Avoid Neighbor
	static_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidNeighborMethod(
			&OSSteerVehicle::doAvoidNeighbor);
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

void OSSteerVehicle::doUpdateSteerVehicle(const float currentTime,
		const float elapsedTime)
{
	if (mVehicle->speed() > 0.0)
	{
		NodePath ownerObjectNP/* = mOwnerObject->getNodePath()*/;
		LPoint3f updatedPos = ossup::OpenSteerVec3ToLVecBase3f(
				mVehicle->position());
		switch (mMovType)
		{
		case OPENSTEER:
			break;
		case OPENSTEER_KINEMATIC:
		{
			//correct updatedPos.z if needed
//			//ray down XXX
//			mHitResult = mBulletWorld->ray_test_closest(
//					updatedPos + mDeltaRayOrig, updatedPos + mDeltaRayDown,
//					mRayMask);
//			if (mHitResult.has_hit())
//			{
//				//updatedPos.z needs correction
//				updatedPos.set_z(mHitResult.get_hit_pos().get_z());
//				//correct vehicle position
//				mVehicle->setPosition(LVecBase3fToOpenSteerVec3(updatedPos));
//			}
		}
			break;
		default:
			break;
		}
		//correct z if there is a kinematic rigid body
		updatedPos.set_z(updatedPos.get_z() + mCorrectHeightRigidBody);
		//update node path pos
		ownerObjectNP.set_pos(updatedPos);
		//update node path dir
		mUpAxisFixed ?
		//up axis fixed: z
				ownerObjectNP.heads_up(
						updatedPos
								- ossup::OpenSteerVec3ToLVecBase3f(
										mVehicle->forward()), LVector3f::up()) :
				//up axis free: from mVehicle
				ownerObjectNP.heads_up(
						updatedPos
								- ossup::OpenSteerVec3ToLVecBase3f(
										mVehicle->forward()),
						ossup::OpenSteerVec3ToLVecBase3f(mVehicle->up()));

		//handle Move/Steady events
		//throw Move event (if enabled)
		if (mMove.mEnable)
		{
			doThrowEvent(mMove);
		}
		//reset Steady event (if enabled and if thrown)
		if (mSteady.mEnable && mSteady.mThrown)
		{
			mSteady.mThrown = false;
			mSteady.mTimeElapsed = 0.0;
		}
	}
	else
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
			doThrowEvent(mSteady);
		}
	}

	//handle SteerLibrary events
	doHandleSteerLibraryEvent(mPathFollowing, mPFCallbackCalled);
	doHandleSteerLibraryEvent(mAvoidObstacle, mAOCallbackCalled);
	doHandleSteerLibraryEvent(mAvoidCloseNeighbor, mACNCallbackCalled);
	doHandleSteerLibraryEvent(mAvoidNeighbor, mANCallbackCalled);
}

void OSSteerVehicle::doExternalUpdateSteerVehicle(const float currentTime,
		const float elapsedTime)
{
	NodePath ownerNP/* = mOwnerObject->getNodePath()*/;
	OpenSteer::Vec3 oldPos = mVehicle->position();
	//update vehicle's
	//position,
	mVehicle->setPosition(
			ossup::LVecBase3fToOpenSteerVec3(
					ownerNP.get_pos()
							- LVector3f(0.0, 0.0, mCorrectHeightRigidBody)));
	//forward,
	mVehicle->setForward(
			ossup::LVecBase3fToOpenSteerVec3(
					ownerNP.get_parent().get_relative_vector(ownerNP,
							-LVector3f::forward())).normalize());
	//up,
	mVehicle->setUp(
			ossup::LVecBase3fToOpenSteerVec3(
					ownerNP.get_parent().get_relative_vector(ownerNP,
							LVector3f::up())).normalize());
	//side,
	mVehicle->setUnitSideFromForwardAndUp();
	//speed (elapsedTime should be != 0)
	mVehicle->setSpeed((mVehicle->position() - oldPos).length() / elapsedTime);
	//
	//no event thrown: external updating sub-system will do, if expected
}

void OSSteerVehicle::doEnableSteerVehicleEvent(EventThrown event,
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

void OSSteerVehicle::doPathFollowing(const OpenSteer::Vec3& future,
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

void OSSteerVehicle::doAvoidObstacle(const float minDistanceToCollision)
{
	//handle Avoid Obstacle event
	if (mAvoidObstacle.mEnable)
	{
		doThrowEvent(mAvoidObstacle);
		//set the flag
		mAOCallbackCalled = true;
	}
}

void OSSteerVehicle::doAvoidCloseNeighbor(
		const OpenSteer::AbstractVehicle& other, const float additionalDistance)
{
	//handle Avoid Close Neighbor event
	if (mAvoidCloseNeighbor.mEnable)
	{
		doThrowEvent(mAvoidCloseNeighbor);
		//set the flag
		mACNCallbackCalled = true;
	}
}

void OSSteerVehicle::doAvoidNeighbor(const OpenSteer::AbstractVehicle& threat,
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

void OSSteerVehicle::doThrowEvent(ThrowEventData& eventData)
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

void OSSteerVehicle::doHandleSteerLibraryEvent(ThrowEventData& eventData,
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

//TypedObject semantics: hardcoded
TypeHandle OSSteerVehicle::_type_handle;
