/**
 * \file osSteerManager.h
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERMANGER_H_
#define OSSTEERMANGER_H_

#include "osTools.h"
#include "opensteer_includes.h"
#include "collisionTraverser.h"
#include "collisionHandlerQueue.h"
#include "collisionRay.h"

class OSSteerPlugIn;
class OSSteerVehicle;

/**
 * OSSteerManager Singleton class.
 *
 * Used for handling OSSteerPlugIns and OSSteerVehicles.
 */
class EXPORT_CLASS OSSteerManager: public TypedReferenceCount,
		public Singleton<OSSteerManager>
{
PUBLISHED:
	OSSteerManager(const NodePath& root = NodePath(),
			const CollideMask& mask = GeomNode::get_default_collide_mask());
	virtual ~OSSteerManager();

	// OSSteerPlugIns
	NodePath create_steer_plug_in();
	bool destroy_steer_plug_in(NodePath plugInNP);
	NodePath get_steer_plug_in(int index) const;
	INLINE int get_num_steer_plug_ins() const;
	MAKE_SEQ(get_steer_plug_ins, get_num_steer_plug_ins, get_steer_plug_in);

	// OSSteerVehicles
	NodePath create_steer_vehicle(const string& name);
	bool destroy_steer_vehicle(NodePath steerVehicleNP);
	NodePath get_steer_vehicle(int index) const;
	INLINE int get_num_steer_vehicles() const;
	MAKE_SEQ(get_steer_vehicles, get_num_steer_vehicles, get_steer_vehicle);

	/**
	 * The type of object for creation parameters.
	 */
	enum OSType
	{
		STEERPLUGIN = 0,
		STEERVEHICLE
	};

	ValueListString get_parameter_name_list(OSType type);
	void set_parameter_values(OSType type, const string& paramName, const ValueListString& paramValues);
	ValueListString get_parameter_values(OSType type, const string& paramName);
	void set_parameter_value(OSType type, const string& paramName, const string& value);
	string get_parameter_value(OSType type, const string& paramName);
	void set_parameters_defaults(OSType type);

	AsyncTask::DoneStatus update(GenericAsyncTask* task);
	void start_default_update();
	void stop_default_update();

	//Get singleton
	INLINE static OSSteerManager* get_global_ptr();

	//Utilities
	float get_bounding_dimensions(NodePath modelNP, LVecBase3f& modelDims,
			LVector3f& modelDeltaCenter);
	PairBoolFloat get_collision_height(const LPoint3f& origin,
			const NodePath& space = NodePath());
	INLINE CollideMask get_collide_mask();
	INLINE NodePath get_collision_root();
	INLINE CollisionTraverser* get_collision_traverser();
	INLINE CollisionHandlerQueue* get_collision_handler();
	INLINE CollisionRay* get_collision_ray();

	//serialization
	bool write_to_bam_file(const string& fileName);
	bool read_from_bam_file(const string& fileName);

private:
	///List of OSSteerPlugIns handled by this manager.
	typedef pvector<PT(OSSteerPlugIn)> SteerPlugInList;
	SteerPlugInList mNavMeshes;
	///OSSteerPlugIns' parameter table.
	ParameterTable mNavMeshesParameterTable;

	///List of OSSteerVehicles handled by this template.
	typedef pvector<PT(OSSteerVehicle)> SteerVehicleList;
	SteerVehicleList mCrowdAgents;
	///OSSteerVehicles' parameter table.
	ParameterTable mCrowdAgentsParameterTable;

	///@{
	///A task data for step simulation update.
	PT(TaskInterface<OSSteerManager>::TaskData) mUpdateData;
	PT(AsyncTask) mUpdateTask;
	///@}

	///Utilities.
	NodePath mRoot;
	CollideMask mMask; //a.k.a. BitMask32
	CollisionTraverser* mCTrav;
	CollisionHandlerQueue* mCollisionHandler;
	CollisionRay* mPickerRay;

	///TypedObject semantics: hardcoded
public:
	static TypeHandle get_class_type()
	{
		return _type_handle;
	}
	static void init_type()
	{
		TypedReferenceCount::init_type();
		register_type(_type_handle, "OSSteerManager",
				TypedReferenceCount::get_class_type());
	}
	virtual TypeHandle get_type() const override
	{
		return get_class_type();
	}
	virtual TypeHandle force_init_type() override
	{
		init_type();
		return get_class_type();
	}

private:
	static TypeHandle _type_handle;

};

///inline
#include "osSteerManager.I"

#endif /* OSSTEERMANGER_H_ */
