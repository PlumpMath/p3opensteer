/**
 * \file osSteerManager.cxx
 *
 * \date 2016-05-13
 * \author consultit
 */

#include "osSteerPlugIn.h"
#include "osSteerVehicle.h"
#include "asyncTaskManager.h"
#include "bamFile.h"
#include "osSteerManager.h"

///OSSteerManager
/**
 *
 */
OSSteerManager::OSSteerManager(const NodePath& root, const CollideMask& mask) :
		mReferenceNP(NodePath("ReferenceNode")), mRoot(root), mMask(mask), mCollisionHandler(
				NULL), mPickerRay(NULL), mCTrav(
		NULL), mReferenceDebugNP(NodePath("ReferenceDebugNode")), mReferenceDebug2DNP(
				NodePath("ReferenceDebugNode2D")), mRef(0)
{
	PRINT_DEBUG(
			"OSSteerManager::OSSteerManager: creating the singleton manager.");

	mSteerPlugIns.clear();
	mSteerPlugInsParameterTable.clear();
	mSteerVehicles.clear();
	mSteerVehiclesParameterTable.clear();
	mObstacles.first().clear();
	mObstacles.second().clear();
	set_parameters_defaults(STEERPLUGIN);
	set_parameters_defaults(STEERVEHICLE);
	//
	mUpdateData.clear();
	mUpdateTask.clear();
	//
	if (! mRoot.is_empty())
	{
		mCTrav = new CollisionTraverser();
		mCollisionHandler = new CollisionHandlerQueue();
		mPickerRay = new CollisionRay();
		PT(CollisionNode)pickerNode = new CollisionNode(string("OSSteerManager::pickerNode"));
		pickerNode->add_solid(mPickerRay);
		pickerNode->set_from_collide_mask(mMask);
		pickerNode->set_into_collide_mask(BitMask32::all_off());
		mCTrav->add_collider(mRoot.attach_new_node(pickerNode),
				mCollisionHandler);
	}
#ifdef OS_DEBUG
	mDD = NULL;
	if (!mRoot.is_empty())
	{
		//create new DebugDrawer
		mDD = new ossup::DebugDrawPanda3d(mRoot);
	}
#endif //OS_DEBUG
}

/**
 *
 */
OSSteerManager::~OSSteerManager()
{
	PRINT_DEBUG("OSSteerManager::~OSSteerManager: destroying the singleton manager.");

	//stop any default update
	stop_default_update();
	//destroy all OSSteerVehicles
	PTA(PT(OSSteerVehicle))::iterator iterC = mSteerVehicles.begin();
	while (iterC != mSteerVehicles.end())
	{
		//\see http://stackoverflow.com/questions/596162/can-you-remove-elements-from-a-stdlist-while-iterating-through-it
		//give a chance to OSSteerVehicle to cleanup itself before being destroyed.
		(*iterC)->do_finalize();
		//remove the OSSteerVehicles from the inner list (and from the update task)
		mSteerVehicles.erase(iterC);
	}

	//destroy all OSSteerPlugIns
	PTA(PT(OSSteerPlugIn))::iterator iterN = mSteerPlugIns.begin();
	while (iterN != mSteerPlugIns.end())
	{
		//\see http://stackoverflow.com/questions/596162/can-you-remove-elements-from-a-stdlist-while-iterating-through-it
		//give a chance to OSSteerVehicle to cleanup itself before being destroyed.
		(*iterN)->do_finalize();
		//remove the OSSteerVehicles from the inner list (and from the update task)
		mSteerPlugIns.erase(iterN);
	}

	//clear parameters' tables
	mSteerPlugInsParameterTable.clear();
	mSteerVehiclesParameterTable.clear();
	//
	delete mCTrav;

#ifdef OS_DEBUG
	if (mDD)
	{
		delete mDD;
		mDD = NULL;
	}
#endif //OS_DEBUG
}

/**
 * Creates a OSSteerPlugIn.
 * Returns a NodePath to the new OSSteerPlugIn,or an empty NodePath with the
 * ET_fail error type set on error.
 */
NodePath OSSteerManager::create_steer_plug_in()
{
	PT(OSSteerPlugIn) newSteerPlugIn = new OSSteerPlugIn();
	nassertr_always(newSteerPlugIn, NodePath::fail())

	// set reference nodes
	newSteerPlugIn->mReferenceNP = mReferenceNP;
	newSteerPlugIn->mReferenceDebugNP = mReferenceDebugNP;
	newSteerPlugIn->mReferenceDebug2DNP = mReferenceDebug2DNP;
	// initialize the new SteerPlugIn
	newSteerPlugIn->do_initialize();

	// add the new SteerPlugIn to the inner list (and to the update task)
	mSteerPlugIns.push_back(newSteerPlugIn);
	// reparent to reference node
	NodePath np = mReferenceNP.attach_new_node(newSteerPlugIn);
	//
	return np;
}

/**
 * Destroys a OSSteerPlugIn.
 * Returns false on error.
 */
bool OSSteerManager::destroy_steer_plug_in(NodePath steerPlugInNP)
{
	CONTINUE_IF_ELSE_R(
			steerPlugInNP.node()->is_of_type(OSSteerPlugIn::get_class_type()),
			false)

	PT(OSSteerPlugIn)steerPlugIn = DCAST(OSSteerPlugIn, steerPlugInNP.node());
	SteerPlugInList::iterator iter = find(mSteerPlugIns.begin(),
			mSteerPlugIns.end(), steerPlugIn);
	CONTINUE_IF_ELSE_R(iter != mSteerPlugIns.end(), false)

	//give a chance to OSSteerPlugIn to cleanup itself before being destroyed.
	steerPlugIn->do_finalize();

	//remove the OSSteerPlugIn from the inner list (and from the update task)
	mSteerPlugIns.erase(iter);
	//
	return true;
}

/**
 * Gets a NodePath to OSSteerPlugIn by index.
 * Return an empty NodePath with the ET_fail error type set on error.
 */
NodePath OSSteerManager::get_steer_plug_in(int index) const
{
	nassertr_always((index >= 0) && (index < (int ) mSteerPlugIns.size()),
			NodePath::fail())

	return NodePath::any_path(mSteerPlugIns[index]);
}

/**
 * Creates a OSSteerVehicle with a given (mandatory and not empty) name.
 * Returns a NodePath to the new OSSteerVehicle,or an empty NodePath with the
 * ET_fail error type set on error.
 */
NodePath OSSteerManager::create_steer_vehicle(const string& name)
{
	nassertr_always(! name.empty(), NodePath::fail())

	PT(OSSteerVehicle) newSteerVehicle = new OSSteerVehicle(name);
	nassertr_always(newSteerVehicle, NodePath::fail())

	// set reference node
	newSteerVehicle->mReferenceNP = mReferenceNP;
	//initialize the new SteerVehicle
	newSteerVehicle->do_initialize();

	//add the new SteerVehicle to the inner list
	mSteerVehicles.push_back(newSteerVehicle);
	// reparent to reference node
	NodePath np = mReferenceNP.attach_new_node(newSteerVehicle);
	//
	return np;
}

/**
 * Destroys a OSSteerVehicle.
 * Returns false on error.
 */
bool OSSteerManager::destroy_steer_vehicle(NodePath steerVehicleNP)
{
	CONTINUE_IF_ELSE_R(
			steerVehicleNP.node()->is_of_type(OSSteerVehicle::get_class_type()),
			false)

	PT(OSSteerVehicle)steerVehicle = DCAST(OSSteerVehicle, steerVehicleNP.node());
	SteerVehicleList::iterator iter = find(mSteerVehicles.begin(),
			mSteerVehicles.end(), steerVehicle);
	CONTINUE_IF_ELSE_R(iter != mSteerVehicles.end(), false)

	//give a chance to OSSteerVehicle to cleanup itself before being destroyed.
	steerVehicle->do_finalize();
	//remove the OSSteerVehicle from the inner list (and from the update task)
	mSteerVehicles.erase(iter);
	//
	return true;
}

/**
 * Gets a NodePath to OSSteerVehicle by index.
 * Return an empty NodePath with the ET_fail error type set on error.
 */
NodePath OSSteerManager::get_steer_vehicle(int index) const
{
	nassertr_always((index >= 0) && (index < (int ) mSteerVehicles.size()),
			NodePath::fail())

	return NodePath::any_path(mSteerVehicles[index]);
}

/**
 * Sets a multi-valued parameter to a multi-value overwriting the existing one(s).
 */
void OSSteerManager::set_parameter_values(OSType type, const string& paramName,
		const ValueList<string>& paramValues)
{
	pair<ParameterTableIter, ParameterTableIter> iterRange;
	if (type == STEERPLUGIN)
	{
		//find from mParameterTable the paramName's values to be overwritten
		iterRange = mSteerPlugInsParameterTable.equal_range(paramName);
		//...and erase them
		mSteerPlugInsParameterTable.erase(iterRange.first, iterRange.second);
		//insert the new values
		for (int idx = 0; idx < paramValues.size(); ++idx)
		{
			mSteerPlugInsParameterTable.insert(
					ParameterNameValue(paramName, paramValues[idx]));
		}
	}
	else if (type == STEERVEHICLE)
	{
		//find from mParameterTable the paramName's values to be overwritten
		iterRange = mSteerVehiclesParameterTable.equal_range(paramName);
		//...and erase them
		mSteerVehiclesParameterTable.erase(iterRange.first, iterRange.second);
		//insert the new values
		for (int idx = 0; idx < paramValues.size(); ++idx)
		{
			mSteerVehiclesParameterTable.insert(
					ParameterNameValue(paramName, paramValues[idx]));
		}
	}
}

/**
 * Gets the multiple values of a (actually set) parameter.
 */
ValueList<string> OSSteerManager::get_parameter_values(OSType type,
		const string& paramName) const
{
	ValueList<string> strList;
	ParameterTableConstIter iter;
	pair<ParameterTableConstIter, ParameterTableConstIter> iterRange;
	if (type == STEERPLUGIN)
	{
		iterRange = mSteerPlugInsParameterTable.equal_range(paramName);
		if (iterRange.first != iterRange.second)
		{
			for (iter = iterRange.first; iter != iterRange.second; ++iter)
			{
				strList.add_value(iter->second);
			}
		}
	}
	else if (type == STEERVEHICLE)
	{
		iterRange = mSteerVehiclesParameterTable.equal_range(paramName);
		if (iterRange.first != iterRange.second)
		{
			for (iter = iterRange.first; iter != iterRange.second; ++iter)
			{
				strList.add_value(iter->second);
			}
		}
	}
	//
	return strList;
}

/**
 * Sets a multi/single-valued parameter to a single value overwriting the existing one(s).
 */
void OSSteerManager::set_parameter_value(OSType type, const string& paramName,
		const string& value)
{
	ValueList<string> valueList;
	valueList.add_value(value);
	set_parameter_values(type, paramName, valueList);
}

/**
 * Gets a single value (i.e. the first one) of a parameter.
 */
string OSSteerManager::get_parameter_value(OSType type,
		const string& paramName) const
{
	ValueList<string> valueList = get_parameter_values(type, paramName);
	return (valueList.size() != 0 ? valueList[0] : string(""));
}

/**
 * Gets a list of the names of the parameters actually set.
 */
ValueList<string> OSSteerManager::get_parameter_name_list(OSType type) const
{
	ValueList<string> strList;
	ParameterTableIter iter;
	ParameterTable tempTable;
	if (type == STEERPLUGIN)
	{
		tempTable = mSteerPlugInsParameterTable;
		for (iter = tempTable.begin(); iter != tempTable.end(); ++iter)
		{
			string name = (*iter).first;
			if (!strList.has_value(name))
			{
				strList.add_value(name);
			}
		}
	}
	else if (type == STEERVEHICLE)
	{
		tempTable = mSteerVehiclesParameterTable;
		for (iter = tempTable.begin(); iter != tempTable.end(); ++iter)
		{
			string name = (*iter).first;
			if (!strList.has_value(name))
			{
				strList.add_value(name);
			}
		}

	}
	//
	return strList;
}

/**
 * Sets all parameters to their default values (if any).
 * \note: After reading objects from bam files, the objects' creation parameters
 * which reside in the manager, are reset to their default values.
 */
void OSSteerManager::set_parameters_defaults(OSType type)
{
	if (type == STEERPLUGIN)
	{
		///mSteerPlugInsParameterTable must be the first cleared
		mSteerPlugInsParameterTable.clear();
		//sets the (mandatory) parameters to their default values:
		//sets the (mandatory) parameters to their default values:
		mSteerPlugInsParameterTable.insert(
				ParameterNameValue("plugin_type", "one_turning"));
		mSteerPlugInsParameterTable.insert(
				ParameterNameValue("pathway",
						"0.0,0.0,0.0:1.0,1.0,1.0$1.0$false"));
	}
	else if (type == STEERVEHICLE)
	{
		///mSteerVehiclesParameterTable must be the first cleared
		mSteerVehiclesParameterTable.clear();
		//sets the (mandatory) parameters to their default values:
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("vehicle_type", "one_turning"));
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("external_update", "false"));
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("mov_type", "opensteer"));
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("up_axis_fixed", "false"));
		mSteerVehiclesParameterTable.insert(ParameterNameValue("mass", "1.0"));
		mSteerVehiclesParameterTable.insert(ParameterNameValue("speed", "0.0"));
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("max_force", "0.1"));
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("max_speed", "1.0"));
		mSteerVehiclesParameterTable.insert(
				ParameterNameValue("radius", "1.0"));
	}
}

/**
 * Updates OSSteerPlugIns and their OSSteerVehicles.
 *
 * Will be called automatically in a task.
 */
AsyncTask::DoneStatus OSSteerManager::update(GenericAsyncTask* task)
{
	float dt = ClockObject::get_global_clock()->get_dt();

#ifdef TESTING
	dt = 0.016666667; //60 fps
#endif

	// call all audio components update functions, passing delta time
	for (PTA(PT(OSSteerPlugIn))::size_type index = 0;
			index < mSteerPlugIns.size(); ++index)
	{
		mSteerPlugIns[index]->update(dt);
	}
	//
	return AsyncTask::DS_cont;
}

/**
 * Adds a task to repeatedly call OSSteerPlugIns' updates.
 */
void OSSteerManager::start_default_update()
{
	//create the task for updating AI components
	mUpdateData = new TaskInterface<OSSteerManager>::TaskData(this,
			&OSSteerManager::update);
	mUpdateTask = new GenericAsyncTask(string("OSSteerManager::update"),
			&TaskInterface<OSSteerManager>::taskFunction,
			reinterpret_cast<void*>(mUpdateData.p()));
	//Adds mUpdateTask to the active queue.
	AsyncTaskManager::get_global_ptr()->add(mUpdateTask);
}

/**
 * Removes a task to repeatedly call OSSteerPlugIns' updates.
 */
void OSSteerManager::stop_default_update()
{
	if (mUpdateTask)
	{
		AsyncTaskManager::get_global_ptr()->remove(mUpdateTask);
	}
	//
	mUpdateData.clear();
	mUpdateTask.clear();
}

/**
 * Returns the obstacle's settings with the specified unique reference (>0).
 * Returns OSObstacleSettings::ref == a negative number on error.
 */
OSObstacleSettings OSSteerManager::get_obstacle_settings(int ref) const
{
	OSObstacleSettings settings = OSObstacleSettings();
	settings.set_ref(OS_ERROR);
	CONTINUE_IF_ELSE_R(ref > 0, settings)

	// find settings by ref
	pvector<ObstacleAttributes>& obstacleAttrs =
			const_cast<GlobalObstacles&>(mObstacles).second();
	pvector<ObstacleAttributes>::iterator iter;
	for (iter = obstacleAttrs.begin(); iter != obstacleAttrs.end(); ++iter)
	{
		if ((*iter).first().get_ref() == ref)
		{
			settings = (*iter).first();
			break;
		}
	}
	//
	return settings;
}

/**
 * Returns the NodePath of the obstacle with the specified unique reference (>0).
 * Return an empty NodePath with the ET_fail error type set on error.
 */
NodePath OSSteerManager::get_obstacle_by_ref(int ref) const
{
	NodePath obstacleNP = NodePath::fail();
	CONTINUE_IF_ELSE_R(ref > 0, obstacleNP)

	// find settings by ref
	pvector<ObstacleAttributes>& obstacleAttrs =
			const_cast<GlobalObstacles&>(mObstacles).second();
	pvector<ObstacleAttributes>::iterator iter;
	for (iter = obstacleAttrs.begin(); iter != obstacleAttrs.end(); ++iter)
	{
		if ((*iter).first().get_ref() == ref)
		{
			obstacleNP = (*iter).second();
			break;
		}
	}
	return obstacleNP;
}

/**
 * Gets bounding dimensions of a model NodePath.
 * Puts results into the out parameters: modelDims, modelDeltaCenter and returns
 * modelRadius.
 * - modelDims = absolute dimensions of the model
 * - modelCenter + modelDeltaCenter = origin of coordinate system
 * - modelRadius = radius of the containing sphere
 */
float OSSteerManager::get_bounding_dimensions(NodePath modelNP,
		LVecBase3f& modelDims, LVector3f& modelDeltaCenter) const
{
	//get "tight" dimensions of model
	LPoint3f minP, maxP;
	modelNP.calc_tight_bounds(minP, maxP);
	//
	LVecBase3 delta = maxP - minP;
	LVector3f deltaCenter = -(minP + delta / 2.0);
	//
	modelDims.set(abs(delta.get_x()), abs(delta.get_y()), abs(delta.get_z()));
	modelDeltaCenter.set(deltaCenter.get_x(), deltaCenter.get_y(),
			deltaCenter.get_z());
	float modelRadius = max(max(modelDims.get_x(), modelDims.get_y()),
			modelDims.get_z()) / 2.0;
	return modelRadius;
}

/**
 * Throws a ray downward (-z direction default) from rayOrigin.
 * If collisions are found returns a Pair<bool,float> == (true, height),
 * with height equal to the z-value of the first one.
 * If collisions are not found returns a Pair<bool,float> == (false, 0.0).
 */
Pair<bool,float> OSSteerManager::get_collision_height(const LPoint3f& rayOrigin,
		const NodePath& space) const
{
	//traverse downward starting at rayOrigin
	mPickerRay->set_direction(LVecBase3f(0.0, 0.0, -1.0));
	mPickerRay->set_origin(rayOrigin);
	mCTrav->traverse(mRoot);
	if (mCollisionHandler->get_num_entries() > 0)
	{
		mCollisionHandler->sort_entries();
		CollisionEntry *entry0 =
				mCollisionHandler->get_entry(0);
		LPoint3f target = entry0->get_surface_point(space);
		float collisionHeight = target.get_z();
		return Pair<bool,float>(true, collisionHeight);
	}
	//
	return Pair<bool,float>(false, 0.0);
}

/**
 * Draws the specified primitive, given the points, the color (RGBA) and point's size.
 */
void OSSteerManager::debug_draw_primitive(OSDebugDrawPrimitives primitive,
		const ValueList<LPoint3f>& points, const LVecBase4f color, float size)
{
#ifdef OS_DEBUG
	mDD->begin((ossup::DrawMeshDrawer::DrawPrimitive)primitive, size);
	// calculate the real point list size
	unsigned int realSize;
	switch (primitive)
	{
	case POINTS:
		realSize = points.size();
		break;
	case LINES:
		realSize = points.size() - (points.size() % 2);
		break;
	case TRIS:
		realSize = points.size() - (points.size() % 3);
		break;
	case QUADS:
		realSize = points.size() - (points.size() % 4);
		break;
	default:
		break;
	}
	for (unsigned int i = 0; i < realSize; ++i)
	{
		mDD->vertex(points[i], color);
	}
	mDD->end();
#endif //RN_DEBUG
}

/**
 * Erases all primitives drawn until now.
 */
void OSSteerManager::debug_draw_reset()
{
#ifdef RN_DEBUG
	mDD->reset();
#endif //RN_DEBUG
}

/**
 * Writes to a bam file the entire collections of steer plug-ins, steer vehicles
 * and related geometries (i.e. models' NodePaths)
 */
bool OSSteerManager::write_to_bam_file(const string& fileName)
{
	string errorReport;
	// write to bam file
	BamFile outBamFile;
	if (outBamFile.open_write(Filename(fileName)))
	{
		cout << "Current system Bam version: "
				<< outBamFile.get_current_major_ver() << "."
				<< outBamFile.get_current_minor_ver() << endl;
		// just write the reference node
		if (!outBamFile.write_object(mReferenceNP.node()))
		{
			errorReport += string("Error writing ") + mReferenceNP.get_name()
					+ string(" node in ") + fileName + string("\n");
		}
		// close the file
		outBamFile.close();
	}
	else
	{
		errorReport += string("\nERROR: cannot open ") + fileName;
	}
	//check
	if (errorReport.empty())
	{
		cout
				<< "SUCCESS: all steer plug-in and steer vehicle collections were written to "
				<< fileName << endl;
	}
	else
	{
		cerr << errorReport << endl;
	}
	return errorReport.empty();
}

/**
 * Reads from a bam file the entire hierarchy of steer plug-ins, steer vehicles
 * and related geometries (i.e. models' NodePaths)
 */
bool OSSteerManager::read_from_bam_file(const string& fileName)
{
	string errorReport;
	//read from bamFile
	BamFile inBamFile;
	if (inBamFile.open_read(Filename(fileName)))
	{
		cout << "Current system Bam version: "
				<< inBamFile.get_current_major_ver() << "."
				<< inBamFile.get_current_minor_ver() << endl;
		cout << "Bam file version: " << inBamFile.get_file_major_ver() << "."
				<< inBamFile.get_file_minor_ver() << endl;
		// just read the reference node
		TypedWritable* reference = inBamFile.read_object();
		if (reference)
		{
			//resolve pointers
			if (!inBamFile.resolve())
			{
				errorReport += string("Error resolving pointers in ") + fileName
						+ string("\n");
			}
		}
		else
		{
			errorReport += string("Error reading ") + fileName + string("\n");
		}
		// close the file
		inBamFile.close();
		// restore reference node
		mReferenceNP = NodePath::any_path(DCAST(PandaNode, reference));
	}
	else
	{
		errorReport += string("\nERROR: cannot open ") + fileName;
	}
	//check
	if (errorReport.empty())
	{
		cout << "SUCCESS: all steer plug-ins and steer vehicles were read from "
				<< fileName << endl;
	}
	else
	{
		cerr << errorReport << endl;
	}
	return errorReport.empty();
}

//TypedObject semantics: hardcoded
TypeHandle OSSteerManager::_type_handle;
