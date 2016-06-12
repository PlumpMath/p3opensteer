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
//#include "Game/GamePhysicsManager.h"

//VehicleAddOn typedef.
typedef ossup::VehicleAddOnMixin<ossup::SimpleVehicle, OSSteerVehicle> VehicleAddOn;

OSSteerVehicle::OSSteerVehicle(const string& name):PandaNode(name)
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
	//
	std::string param;
	float value;
	//external update
	mExternalUpdate = (
			mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("external_update"))
			== std::string("true") ? true : false);
	//type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("type"));
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
//	//register to SteerPlugIn objectId
//	mSteerPlugInObjectId = ObjectId(
//			mTmpl->parameter(std::string("add_to_plugin")));
	//mov type
	param = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("mov_type"));
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
			mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("up_axis_fixed"))
			== std::string("true") ? true : false);
	//get settings
	ossup::VehicleSettings settings;
	//mass
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("mass")).c_str(),
			NULL);
	settings.m_mass = (value >= 0.0 ? value : 1.0);
	//radius
	mInputRadius = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("radius")).c_str(),
			NULL);
	//speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("speed")).c_str(),
			NULL);
	settings.m_speed = (value >= 0.0 ? value : -value);
	//max force
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("max_force")).c_str(),
			NULL);
	settings.m_maxForce = (value >= 0.0 ? value : -value);
	//max speed
	value = STRTOF(mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("max_speed")).c_str(),
			NULL);
	settings.m_maxSpeed = (value >= 0.0 ? value : 1.0);
//	//ray mask
//	param = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("ray_mask"));
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
	dynamic_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
	//thrown events
	mThrownEventsParam = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN, std::string("thrown_events"));
	//
	LVecBase3f modelDims;
	LVector3f modelDeltaCenter;
	float modelRadius;
//	GamePhysicsManager::GetSingletonPtr()->getBoundingDimensions(
//			mOwnerObject->getNodePath(), modelDims, modelDeltaCenter,
//			modelRadius);
	//set definitive radius
	if (mInputRadius <= 0.0)
	{
		// store new radius into settings
		ossup::VehicleSettings settings =
				dynamic_cast<VehicleAddOn*>(mVehicle)->getSettings();
		settings.m_radius = modelRadius;
		dynamic_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
	}
//	//set physics parameters
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
	dynamic_cast<VehicleAddOn*>(mVehicle)->setEntity(this);
	//
	! mExternalUpdate ?
			dynamic_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
					&OSSteerVehicle::doUpdateSteerVehicle) :
			dynamic_cast<VehicleAddOn*>(mVehicle)->setEntityUpdateMethod(
					&OSSteerVehicle::doExternalUpdateSteerVehicle);

//	//set the bullet physics
//	mBulletWorld = GamePhysicsManager::GetSingletonPtr()->bulletWorld();

	//set thrown events if any
	unsigned int idx1, valueNum1;
	std::vector<std::string> paramValuesStr1, paramValuesStr2;
	if (mThrownEventsParam != std::string(""))
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
//				std::string objectType = std::string(
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
//						name = objectType + "_SteerVehicle_Move";
					}
				}
				else if (paramValuesStr2[0] == "steady")
				{
					event = STEADYEVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_Steady";
					}
				}
				else if (paramValuesStr2[0] == "path_following")
				{
					event = PATHFOLLOWINGEVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_PathFollowing";
					}
				}
				else if (paramValuesStr2[0] == "avoid_obstacle")
				{
					event = AVOIDOBSTACLEEVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_AvoidObstacle";
					}
				}
				else if (paramValuesStr2[0] == "avoid_close_neighbor")
				{
					event = AVOIDCLOSENEIGHBOREVENT;
					//check name
					if (name == "")
					{
						//set default name
//						name = objectType + "_SteerVehicle_AvoidCloseNeighbor";
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
				doEnableSteerVehicleEvent(event, eventData);
			}
		}
	}

	//set the callbacks
	//Path Following
	dynamic_cast<VehicleAddOn*>(mVehicle)->setEntityPathFollowingMethod(
			&OSSteerVehicle::doPathFollowing);
	//Avoid Obstacle
	dynamic_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidObstacleMethod(
			&OSSteerVehicle::doAvoidObstacle);
	//Avoid Close Neighbor
	dynamic_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidCloseNeighborMethod(
			&OSSteerVehicle::doAvoidCloseNeighbor);
	//Avoid Neighbor
	dynamic_cast<VehicleAddOn*>(mVehicle)->setEntityAvoidNeighborMethod(
			&OSSteerVehicle::doAvoidNeighbor);
	//clear all no more needed "Param" variables
	mThrownEventsParam.clear();
}

void OSSteerVehicle::onAddToSceneSetup()
{
	//set vehicle's forward,( side,) up and position
	NodePath ownerNP/* = mOwnerObject->getNodePath()*/;
	ossup::VehicleSettings settings =
			dynamic_cast<VehicleAddOn*>(mVehicle)->getSettings();
	settings.m_forward =
			ossup::LVecBase3fToOpenSteerVec3(
					ownerNP.get_parent().get_relative_vector(
							ownerNP, -LVector3f::forward())).normalize();
	settings.m_up = ossup::LVecBase3fToOpenSteerVec3(
			ownerNP.get_parent().get_relative_vector(
					ownerNP, LVector3f::up())).normalize();
	settings.m_position = ossup::LVecBase3fToOpenSteerVec3(
			ownerNP.get_pos());
	dynamic_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);

	//set SteerPlugIn object (if any)
//	PT(Object)steerPlugInObject =
//			ObjectTemplateManager::GetSingleton().getCreatedObject(mSteerPlugInObjectId);
	//Add to SteerPlugIn update
//	if (steerPlugInObject)
//	{
//		SMARTPTR(Component)aiComp = steerPlugInObject->getComponent(componentFamilyType());
//		//
//		if (aiComp and (aiComp->componentType() == ComponentType("SteerPlugIn")))
//		{
//			DCAST(SteerPlugIn, aiComp)->addSteerVehicle(this);
//		}
//	}
}

void OSSteerVehicle::onRemoveFromObjectCleanup()
{
	//
	delete mVehicle;
	do_reset();
}

void OSSteerVehicle::onRemoveFromSceneCleanup()
{
	//Remove from SteerPlugIn update (if previously added)
	//mSteerPlugIn will be cleared during removing, so
	//remove through a temporary pointer
	PT(OSSteerPlugIn) steerPlugIn = mSteerPlugIn;
	if (steerPlugIn)
	{
		steerPlugIn->removeSteerVehicle(this);
	}
}

void OSSteerVehicle::setSettings(const ossup::VehicleSettings& settings)
{
	//set vehicle settings
	dynamic_cast<VehicleAddOn*>(mVehicle)->setSettings(settings);
}

ossup::VehicleSettings OSSteerVehicle::getSettings()
{
	//get vehicle settings
	return dynamic_cast<VehicleAddOn*>(mVehicle)->getSettings();
}

PT(OSSteerPlugIn) OSSteerVehicle::getSteerPlugIn() const
{
	return mSteerPlugIn;
}

void OSSteerVehicle::doUpdateSteerVehicle(const float currentTime,
		const float elapsedTime)
{
	if (mVehicle->speed() > 0.0)
	{
		NodePath ownerObjectNP/* = mOwnerObject->getNodePath()*/;
		LPoint3f updatedPos = ossup::OpenSteerVec3ToLVecBase3f(mVehicle->position());
		switch (mMovType)
		{
		case OPENSTEER:
			break;
		case OPENSTEER_KINEMATIC:
		{
			//correct updatedPos.z if needed
//			//ray down
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
					updatedPos - ossup::OpenSteerVec3ToLVecBase3f(mVehicle->forward()),
					LVector3f::up()):
			//up axis free: from mVehicle
			ownerObjectNP.heads_up(
						updatedPos - ossup::OpenSteerVec3ToLVecBase3f(mVehicle->forward()),
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
					ownerNP.get_parent().get_relative_vector(
							ownerNP, -LVector3f::forward())).normalize());
	//up,
	mVehicle->setUp(
			ossup::LVecBase3fToOpenSteerVec3(
					ownerNP.get_parent().get_relative_vector(
							ownerNP, LVector3f::up())).normalize());
	//side,
	mVehicle->setUnitSideFromForwardAndUp();
	//speed (elapsedTime should be != 0)
	mVehicle->setSpeed((mVehicle->position() - oldPos).length() / elapsedTime);
	//
	//no event thrown: external updating sub-system will do, if expected
}

void OSSteerVehicle::doEnableSteerVehicleEvent(EventThrown event, ThrowEventData eventData)
{
	//some checks
	nassertv_always(! eventData.mEventName.empty())

	if (eventData.mFrequency <= 0.0)
	{
		eventData.mFrequency = 30.0;
	}

	switch (event)
	{
	case MOVEEVENT:
		if(mMove.mEnable != eventData.mEnable)
		{
			mMove = eventData;
			mMove.mTimeElapsed = 0;
		}
		break;
	case STEADYEVENT:
		if(mSteady.mEnable != eventData.mEnable)
		{
			mSteady = eventData;
			mSteady.mTimeElapsed = 0;
		}
		break;
	case PATHFOLLOWINGEVENT:
		if(mPathFollowing.mEnable != eventData.mEnable)
		{
			mPathFollowing = eventData;
			mPathFollowing.mTimeElapsed = 0;
			mPFCallbackCalled = false;
		}
		break;
	case AVOIDOBSTACLEEVENT:
		if(mAvoidObstacle.mEnable != eventData.mEnable)
		{
			mAvoidObstacle = eventData;
			mAvoidObstacle.mTimeElapsed = 0;
			mAOCallbackCalled = false;
		}
		break;
	case AVOIDCLOSENEIGHBOREVENT:
		if(mAvoidCloseNeighbor.mEnable != eventData.mEnable)
		{
			mAvoidCloseNeighbor = eventData;
			mAvoidCloseNeighbor.mTimeElapsed = 0;
			mACNCallbackCalled = false;
		}
		break;
	case AVOIDNEIGHBOREVENT:
		if(mAvoidNeighbor.mEnable != eventData.mEnable)
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

//TypedObject semantics: hardcoded
TypeHandle OSSteerVehicle::_type_handle;
