/**
 * \file osSteerPlugIn.cxx
 *
 * \date 2016-05-13
 * \author consultit
 */

#include "osSteerPlugIn.h"

#include "osSteerVehicle.h"
#include "osSteerManager.h"

#ifndef CPPPARSER
#include "support/PlugIn_OneTurning.h"
#include "support/PlugIn_Pedestrian.h"
#include "support/PlugIn_Boids.h"
#include "support/PlugIn_MultiplePursuit.h"
#include "support/PlugIn_Soccer.h"
#include "support/PlugIn_CaptureTheFlag.h"
#include "support/PlugIn_LowSpeedTurn.h"
#include "support/PlugIn_MapDrive.h"
#endif //CPPPARSER

OSSteerPlugIn::OSSteerPlugIn(const string& name) :
		PandaNode(name)
{
	do_reset();
}

OSSteerPlugIn::~OSSteerPlugIn()
{
}

void OSSteerPlugIn::do_initialize()
{
	WPT(OSSteerManager)mTmpl = OSSteerManager::get_global_ptr();
	//
	//set OSSteerPlugIn parameters (store internally for future use)
	//type
	string mPlugInTypeParam = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN,
			string("type"));
	//pathway (will be used on setup())
	string mPathwayParam = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN,
			string("pathway"));
	//obstacles (will be used on setup())
	plist<string> mObstacleListParam = mTmpl->get_parameter_values(OSSteerManager::STEERPLUGIN,
			string("obstacles"));
	//
	//create the plug in
	if (mPlugInTypeParam == string("pedestrian"))
	{
		mPlugIn = new ossup::PedestrianPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInTypeParam == string("boid"))
	{
		mPlugIn = new ossup::BoidsPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInTypeParam == string("multiple_pursuit"))
	{
		mPlugIn = new ossup::MpPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInTypeParam == string("soccer"))
	{
		mPlugIn = new ossup::MicTestPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInTypeParam == string("capture_the_flag"))
	{
		mPlugIn = new ossup::CtfPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInTypeParam == string("low_speed_turn"))
	{
		mPlugIn = new ossup::LowSpeedTurnPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInTypeParam == string("map_drive"))
	{
		mPlugIn = new ossup::MapDrivePlugIn<OSSteerVehicle>;
	}
	else
	{
		//default: "one_turning"
		mPlugIn = new ossup::OneTurningPlugIn<OSSteerVehicle>;
	}
	//build pathway
	doBuildPathway(mPathwayParam);
	//set the plugin local obstacles reference
	dynamic_cast<ossup::PlugIn*>(mPlugIn)->localObstacles = &mLocalObstacles;
	//set the plugin global obstacles reference
	dynamic_cast<ossup::PlugIn*>(mPlugIn)->obstacles =
			&mTmpl->get_opensteer_obstacles();
	//add its own obstacles
	if (! mReferenceNP.is_empty())
	{
		doAddObstacles(mObstacleListParam);
	}
	//open the plug in
	mPlugIn->open();
}

void OSSteerPlugIn::doBuildPathway(const string& pathwayParam)
{
	//
	pvector<string> paramValues1Str, paramValues2Str, paramValues3Str;
	unsigned int idx, valueNum;
	//build pathway
	paramValues1Str = parseCompoundString(pathwayParam, '$');
	valueNum = paramValues1Str.size();
	if (valueNum != 3)
	{
		//there aren't all mandatory parameters: set hardcoded defaults
		paramValues1Str = parseCompoundString(
				string("0.0,0.0,0.0:1.0,1.0,1.0$1.0$false"), '$');
	}
	//pathway::points (forced to at least 2)
	paramValues2Str = parseCompoundString(paramValues1Str[0], ':');
	valueNum = paramValues2Str.size();
	if (valueNum == 0)
	{
		paramValues2Str = parseCompoundString(
				string("0.0,0.0,0.0:1.0,1.0,1.0"), ':');
	}
	else if (valueNum == 1)
	{
		paramValues2Str.push_back(string("1.0,1.0,1.0"));
	}
	unsigned int numPoints = paramValues2Str.size();
	OpenSteer::Vec3* points = new OpenSteer::Vec3[numPoints];
	for (idx = 0; idx < numPoints; ++idx)
	{
		paramValues3Str = parseCompoundString(paramValues2Str[idx], ',');
		if (paramValues3Str.size() < 3)
		{
			paramValues3Str.resize(3, "0.0");
		}
		LVector3f values;
		for (unsigned int i = 0; i < 3; ++i)
		{
			values[i] = strtof(paramValues3Str[i].c_str(), NULL);
		}
		points[idx] = ossup::LVecBase3fToOpenSteerVec3(values);
	}
	//get pathway::closedCycle
	bool closedCycle =
			(paramValues1Str[2] == string("true") ? true : false);
	//get pathway::radii (forced to at least 1) and set pathway
	paramValues2Str = parseCompoundString(paramValues1Str[1], ':');
	valueNum = paramValues2Str.size();
	if (valueNum == 0)
	{
		paramValues2Str.push_back(string("1.0"));
	}
	unsigned int numRadii = paramValues2Str.size();	//radii specified
	if (numRadii == 1)
	{
		//single radius
		float radius = strtof(paramValues2Str[0].c_str(), NULL);
		if (radius < 0.0)
		{
			radius = -radius;
		}
		else if (radius == 0.0)
		{
			radius = 1.0;
		}
		//set pathway: single radius
		dynamic_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numPoints, points, true,
				&radius, closedCycle);
	}
	else
	{
		//several radii
		unsigned int numRadiiAllocated = (
				closedCycle ? numPoints : numPoints - 1);
		float *radii = new float[numRadiiAllocated];
		for (idx = 0; idx < numRadiiAllocated; ++idx)
		{
			float value;
			if (idx < numRadii)
			{
				value = strtof(paramValues2Str[idx].c_str(), NULL);
				if (value < 0.0)
				{
					value = -value;
				}
				else if (value == 0.0)
				{
					value = 1.0;
				}
			}
			else
			{
				//radii allocated > radii specified
				value = 1.0;
			}
			radii[idx] = value;
		}
		//set pathway: several radius
		dynamic_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numPoints, points, false,
				radii, closedCycle);
		delete[] radii;
	}
	delete[] points;
}

/**
 * Adds the initial set of obstacles.
 * \note Obstacles' NodePaths are searched as descendants of the reference node
 * (and directly reparented to it if necessary).
 */
void OSSteerPlugIn::doAddObstacles(const plist<string>& obstacleListParam)
{
	//
	pvector<string> paramValues1Str, paramValues2Str;
	//add obstacles
	plist<string>::const_iterator iterList;
	for (iterList = obstacleListParam.begin();
			iterList != obstacleListParam.end(); ++iterList)
	{
		//any "obstacles" string is a "compound" one, i.e. could have the form:
		// "objectId1@shape1@seenFromState1:objectId2@shape2@seenFromState2:...:objectIdN@shapeN@seenFromStateN"
		paramValues1Str = parseCompoundString(*iterList, ':');
		pvector<string>::const_iterator iter;
		for (iter = paramValues1Str.begin(); iter != paramValues1Str.end();
				++iter)
		{
			//any obstacle string must have the form:
			//"objectId@shape@seenFromState"
			paramValues2Str = parseCompoundString(*iter, '@');
			if (paramValues2Str.size() < 3)
			{
				continue;
			}
			//get obstacle object: search reference node's descendants
			NodePath obstacleObject = mReferenceNP.find(
					string("**/" + paramValues2Str[0]));
			if (!obstacleObject.is_empty())
			{
				continue;
			}
			//add the obstacle
			add_obstacle(obstacleObject, paramValues2Str[1], paramValues2Str[2]);
		}
	}
}

/**
 * On destruction cleanup.
 * Gives an OSSteerPlugIn the ability to do any cleaning is necessary when
 * destroyed
 */
void OSSteerPlugIn::do_finalize()
{
	//disable debug drawing if enabled
	disable_debug_drawing();
	//remove all local obstacles
	OpenSteer::ObstacleGroup::iterator iterLocal;
	for (iterLocal = mLocalObstacles.begin();
			iterLocal != mLocalObstacles.end(); ++iterLocal)
	{
		//find in global obstacles and remove it
		//get a reference to the global storage
		pvector<OSSteerManager::ObstacleAttributes>& globalObstacles =
				OSSteerManager::get_global_ptr()->get_obstacles();
		pvector<OSSteerManager::ObstacleAttributes>::iterator iter;
		for(iter = globalObstacles.begin(); iter != globalObstacles.end(); ++iter)
		{
			if ((*iter).first().get_obstacle() == (*iterLocal))
			{
				//found: remove it from global obstacles
				globalObstacles.erase(iter);
				break;
			}
		}
		//delete inner OpenSteer obstacle
		delete *iterLocal;
	}
	//clear local obstacles
	mLocalObstacles.clear();
	//remove all handled SteerVehicles (if any) from update
	set<PT(OSSteerVehicle)>::const_iterator iter;
	for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
	{
		//set steerVehicle reference to null
		(*iter)->mSteerPlugIn.clear();
		//do remove from real update list
		dynamic_cast<ossup::PlugIn*>(mPlugIn)->removeVehicle(
				&(*iter)->getAbstractVehicle());
	}

	//close the plug in
	mPlugIn->close();
	//delete the plug in
	delete mPlugIn;
	do_reset();
}

typedef ossup::VehicleAddOnMixin<ossup::SimpleVehicle, OSSteerVehicle> VehicleAddOn;

int OSSteerPlugIn::addSteerVehicle(NodePath steerVehicle)
{
	// continue if steerVehicle is not NULL and  mReferenceNP is not empty
	CONTINUE_IF_ELSE_R(steerVehicle && (!mReferenceNP.is_empty()), OS_ERROR)

	bool result;
//	//return if steerVehicle is destroying or already belongs to any plug in XXX
//	RETURN_ON_COND(steerVehicle->mSteerPlugIn, Result::ERROR)
//
//	//check if SteerVehicle object needs reparenting
//	NodePath steerVehicleObjectNP = steerVehicle->getOwnerObject()->getNodePath();
//	if(mReferenceNP != steerVehicleObjectNP.get_parent())
//	{
//		//reparenting is needed
//		LPoint3f newPos = steerVehicleObjectNP.get_pos(mReferenceNP);
//		LVector3f newForward = mReferenceNP.get_relative_vector(
//		steerVehicleObjectNP, LVector3f::forward());
//		LVector3f newUp = mReferenceNP.get_relative_vector(
//		steerVehicleObjectNP, LVector3f::up());
//		//the SteerVehicle owner object is reparented to the OSSteerPlugIn
//		//object reference node path, updating pos/dir
//		steerVehicleObjectNP.reparent_to(mReferenceNP);
//		steerVehicleObjectNP.set_pos(newPos);
//		steerVehicleObjectNP.heads_up(newPos + newForward, newUp);
//		//OSSteerPlugIn object updates SteerVehicle's pos/dir
//		//wrt its reference node path
//		VehicleSettings settings =
//		dynamic_cast<VehicleAddOn*>(steerVehicle->mVehicle)->getSettings();
//		settings.m_forward = LVecBase3fToOpenSteerVec3(newForward).normalize();
//		settings.m_up = LVecBase3fToOpenSteerVec3(newUp).normalize();
//		settings.m_position = LVecBase3fToOpenSteerVec3(newPos);
//		dynamic_cast<VehicleAddOn*>(steerVehicle->mVehicle)->setSettings(settings);
//	}
//
//	//add to the set of SteerVehicles
//	mSteerVehicles.insert(steerVehicle);
//	//do add to real update list
//	dynamic_cast<PlugIn*>(mPlugIn)->addVehicle(&steerVehicle->getAbstractVehicle());
//	//set steerVehicle reference to this plugin
//	steerVehicle->mOSSteerPlugIn = this;
	//
	return (result = true ? OS_SUCCESS:OS_ERROR);
}

int OSSteerPlugIn::removeSteerVehicle(NodePath steerVehicle)
{
	// continue if steerVehicle is not NULL and  mReferenceNP is not empty
	CONTINUE_IF_ELSE_R(steerVehicle && (!mReferenceNP.is_empty()), OS_ERROR)

//	//return if steerVehicle is destroying or doesn't belong to this plug in XXX
//	RETURN_ON_COND(steerVehicle->mSteerPlugIn != this, Result::ERROR)
//
//	//set steerVehicle reference to null
//	steerVehicle->mOSSteerPlugIn.clear();
//	//do remove from real update list
//	dynamic_cast<ossup::PlugIn*>(mPlugIn)->removeVehicle(&steerVehicle->getAbstractVehicle());
//	//remove from the set of SteerVehicles
//	mSteerVehicles.erase(steerVehicle);
	//
	return OS_SUCCESS;
}

void OSSteerPlugIn::setPathway(int numOfPoints, LPoint3f const points[],
		bool singleRadius, float const radii[], bool closedCycle)
{
	//convert to OpenSteer points
	OpenSteer::Vec3* osPoints = new OpenSteer::Vec3[numOfPoints];
	for (int idx = 0; idx < numOfPoints; ++idx)
	{
		osPoints[idx] = ossup::LVecBase3fToOpenSteerVec3(points[idx]);
	}
	//set the actual path
	dynamic_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numOfPoints, osPoints,
			singleRadius, radii, closedCycle);
	//
	delete[] osPoints;
}

/**
 * Adds an obstacle.
 * If objectNP is not empty then the added underlying obstacle corresponds to
 * this NodePath, which is directly reparented to the reference node, otherwise
 * it only seen by all the underlying plug-ins.
 * Returns the obstacle's unique reference (>0), or a negative number on error.
 */
int OSSteerPlugIn::add_obstacle(NodePath& objectNP,
		const string& type, const string& seenFromState,
		float width, float height,	float depth,
		float radius, const LVector3f& side, const LVector3f& up,
		const LVector3f& forward, const LPoint3f& position)
{
	LPoint3f newPos = position;
	LVector3f newSide = side, newUp = up, newForw = forward;
	if (!objectNP.is_empty())
	{
		//get obstacle dimensions
		LVecBase3f modelDims;
		LVector3f modelDeltaCenter;
		float modelRadius;
//		if (!buildFromBam) XXX
//		{
		//compute new obstacle dimensions
		modelRadius = OSSteerManager::get_global_ptr()->get_bounding_dimensions(
				objectNP, modelDims, modelDeltaCenter);
//		} XXX
//		else
//		{
//			modelRadius = mObstacles[index].first().get_radius();
//			modelDims = mObstacles[index].first().get_dims();
//		}

		//the obstacle is reparented to the RNNavMesh's reference node path
		objectNP.wrt_reparent_to(mReferenceNP);
		//correct obstacle's parameters
		newPos = objectNP.get_pos();
		newForw = mReferenceNP.get_relative_vector(objectNP,
				LVector3f::forward());
		newUp = mReferenceNP.get_relative_vector(objectNP, LVector3f::up());
		newSide = mReferenceNP.get_relative_vector(objectNP,
				LVector3f::right());
		width = modelDims.get_x();
		height = modelDims.get_z();
		depth = modelDims.get_y();
		radius = modelRadius;
	}
	//set seen from state
	OpenSteer::AbstractObstacle::seenFromState seenFS;
	if (seenFromState == "outside")
	{
		seenFS = OpenSteer::AbstractObstacle::outside;
	}
	else if (seenFromState == "inside")
	{
		seenFS = OpenSteer::AbstractObstacle::inside;
	}
	else
	{
		//default: both
		seenFS = OpenSteer::AbstractObstacle::both;
	}
	//create actually the obstacle
	OpenSteer::AbstractObstacle* obstacle = NULL;
	if (type == string("box"))
	{
		ossup::BoxObstacle* box = new ossup::BoxObstacle(width, height, depth);
		obstacle = box;
		box->setSide(ossup::LVecBase3fToOpenSteerVec3(newSide).normalize());
		box->setUp(ossup::LVecBase3fToOpenSteerVec3(newUp).normalize());
		box->setForward(ossup::LVecBase3fToOpenSteerVec3(newForw).normalize());
		box->setPosition(ossup::LVecBase3fToOpenSteerVec3(newPos));
		obstacle->setSeenFrom(seenFS);
	}
	if (type == string("plane"))
	{
		obstacle = new ossup::PlaneObstacle(
				ossup::LVecBase3fToOpenSteerVec3(newSide).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newUp).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newForw).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newPos));
		obstacle->setSeenFrom(seenFS);
	}
	if (type == string("rectangle"))
	{
		obstacle = new ossup::RectangleObstacle(width, height,
				ossup::LVecBase3fToOpenSteerVec3(newSide).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newUp).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newForw).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newPos), seenFS);
	}
	if (type == string("sphere"))
	{
		obstacle = new ossup::SphereObstacle(radius,
				ossup::LVecBase3fToOpenSteerVec3(newPos));
		obstacle->setSeenFrom(seenFS);
	}
	//store obstacle and all settings
	int ref = OS_ERROR;
	if (obstacle)
	{
		//add to local obstacles
		mLocalObstacles.push_back(obstacle);
		//add to global obstacles
		OSObstacleSettings settings;
		settings.set_type(type);
		settings.set_seenFromState(seenFromState);
		settings.set_position(newPos);
		settings.set_forward(newForw);
		settings.set_up(newUp);
		settings.set_side(newSide);
		settings.set_width(width);
		settings.set_height(height);
		settings.set_depth(depth);
		settings.set_radius(radius);
		ref = unique_ref();
		settings.set_ref(ref);
		settings.set_obstacle(obstacle);
		//save into the global storage
		OSSteerManager::get_global_ptr()->get_obstacles().push_back(
				OSSteerManager::ObstacleAttributes(settings, objectNP));
	}
	return ref;
}

/**
 * Removes an obstacle given its unique ref (>0).
 * Returns the NodePath (possibly empty) that was associated to the underlying
 * obstacle just removed, otherwise an empty NodePath with the ET_fail error
 * type set on error.
 * \note Obstacle will be removed only if it was added by this OSSteerPlugIn.
 */
NodePath OSSteerPlugIn::remove_obstacle(int ref)
{
	NodePath resultNP = NodePath::fail();
	//find in global obstacles
	//get a reference to the global storage
	pvector<OSSteerManager::ObstacleAttributes>& globalObstacles =
			OSSteerManager::get_global_ptr()->get_obstacles();
	pvector<OSSteerManager::ObstacleAttributes>::iterator iter;
	for(iter = globalObstacles.begin(); iter != globalObstacles.end(); ++iter)
	{
		if ((*iter).first().get_ref() == ref)
		{
			break;
		}
	}
	//continue only if found
	CONTINUE_IF_ELSE_R(iter != globalObstacles.end(), resultNP)

	//remove only if obstacle has been added by this OSSteerPlugIn
	OpenSteer::ObstacleGroup::iterator iterLocal = find(mLocalObstacles.begin(),
			mLocalObstacles.end(), (*iter).first().get_obstacle());
	if (iterLocal != mLocalObstacles.end())
	{
		//it was actually added
		resultNP = (*iter).second();
		//remove from global obstacles
		globalObstacles.erase(iter);
		//delete inner OpenSteer obstacle
		delete *iterLocal;
		//remove from local obstacles
		mLocalObstacles.erase(iterLocal);
	}
	//
	return resultNP;
}

/**
 * \brief Updates OpenSteer underlying component.
 *
 * Will be called automatically by an ai manager update.
 * @param data The custom data.
 */
void OSSteerPlugIn::update(float dt)
{
	//currentTime
	mCurrentTime += dt;

#ifdef TESTING
	dt = 0.016666667; //60 fps
#endif

#ifdef OS_DEBUG
	{
		if (mEnableDebugDrawUpdate && mDrawer3d && mDrawer2d)
		{
			//unset enableAnnotation
			ossup::enableAnnotation = true;

			//set drawers
			mDrawer3d->initialize();
			mDrawer2d->initialize();
			gDrawer3d = mDrawer3d;
			gDrawer2d = mDrawer2d;

			// invoke PlugIn's Update method
			mPlugIn->update(mCurrentTime, dt);

			// invoke selected PlugIn's Redraw method
			mPlugIn->redraw(mCurrentTime, dt);
			// draw any annotation queued up during selected PlugIn's Update method
			OpenSteer::drawAllDeferredLines();
			OpenSteer::drawAllDeferredCirclesOrDisks();
			mDrawer3d->finalize();
			mDrawer2d->finalize();
		}
		else
		{
			//unset enableAnnotation
			ossup::enableAnnotation = false;
#endif //OS_DEBUG

			// invoke PlugIn's Update method
			mPlugIn->update(mCurrentTime, dt);

#ifdef OS_DEBUG
		}
	}
#endif //OS_DEBUG
}

/**
 * Writes a sensible description of the OSSteerPlugIn to the indicated output
 * stream.
 */
void OSSteerPlugIn::output(ostream &out) const
{
	out << get_type() << " " << get_name();
}

/**
 * Enables the debug drawing, only if nav mesh has been already setup.
 * A camera node path should be passed as argument.
 */
void OSSteerPlugIn::enable_debug_drawing(NodePath debugCamera)
{
#ifdef OS_DEBUG
	CONTINUE_IF_ELSE_V(mDebugCamera.is_empty())

	if ((!debugCamera.is_empty()) &&
			(!debugCamera.find(string("**/+Camera")).is_empty()))
	{
		//set the debug camera
		mDebugCamera = debugCamera;
		//set the debug node as child of mReferenceDebugNP node
		mDrawer3dNP = mReferenceDebugNP.attach_new_node(
				string("OpenSteerDebugNodePath_") + get_name());
		//set the 2D debug node as child of mReferenceDebug2DNP node
		mDrawer2dNP = mReferenceDebug2DNP.attach_new_node(
				string("OpenSteerDebugNodePath2D_") + get_name());
		//set debug node paths
		mDrawer3dNP.set_bin("fixed", 10);
		mDrawer2dNP.set_bin("fixed", 10);
		//by default Debug NodePaths are hidden
		mDrawer3dNP.hide();
		mDrawer2dNP.hide();
		//no collide mask for all Debug NodePaths' children
		mDrawer3dNP.set_collide_mask(BitMask32::all_off());
		mDrawer2dNP.set_collide_mask(BitMask32::all_off());
		//create new Debug Drawers
		mDrawer3d = new ossup::DrawMeshDrawer(mDrawer3dNP, mDebugCamera, 100,
				0.04);
		mDrawer2d = new ossup::DrawMeshDrawer(mDrawer2dNP, mDebugCamera, 50,
				0.04);
	}
#endif //OS_DEBUG
}

/**
 * Disables the debug drawing.
 */
void OSSteerPlugIn::disable_debug_drawing()
{
#ifdef OS_DEBUG
	if (! mDebugCamera.is_empty())
	{
		//set the recast debug camera to empty node path
		mDebugCamera = NodePath();
		//remove the recast debug node paths
		mDrawer3dNP.remove_node();
		mDrawer2dNP.remove_node();
		//reset the DebugDrawers
		if (mDrawer3d)
		{
			delete mDrawer3d;
			mDrawer3d = NULL;
		}
		if (mDrawer2d)
		{
			delete mDrawer2d;
			mDrawer2d = NULL;
		}
	}
#endif //OS_DEBUG
}

/**
 * Enables/disables debugging.
 */
int OSSteerPlugIn::toggle_debug_drawing(bool enable)
{
#ifdef OS_DEBUG
	//continue if mDrawer3dNP and mDrawer2dNP are not empty
	CONTINUE_IF_ELSE_R((!mDrawer3dNP.is_empty()) && (!mDrawer2dNP.is_empty()),
			OS_ERROR)

	if (enable)
	{
		if (mDrawer3dNP.is_hidden())
		{
			mDrawer3dNP.show();
		}
		if (mDrawer2dNP.is_hidden())
		{
			mDrawer2dNP.show();
		}
	}
	else
	{
		if (! mDrawer3dNP.is_hidden())
		{
			mDrawer3dNP.hide();
		}
		if (! mDrawer2dNP.is_hidden())
		{
			mDrawer2dNP.hide();
		}
	}
	//set Debug Draw Update
	mEnableDebugDrawUpdate = enable;
	//clear drawers
	mDrawer3d->clear();
	mDrawer2d->clear();
	//
#endif
	return OS_SUCCESS;
}

//TypedObject semantics: hardcoded
TypeHandle OSSteerPlugIn::_type_handle;
