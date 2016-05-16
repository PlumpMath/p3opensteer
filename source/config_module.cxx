
#include "config_module.h"
#include "dconfig.h"
#include "osSteerVehicle.h"
#include "osSteerPlugIn.h"
#include "osSteerManager.h"


Configure( config_opensteer );
NotifyCategoryDef( opensteer , "");

ConfigureFn( config_opensteer ) {
  init_libopensteer();
}

void
init_libopensteer() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  // Init your dynamic types here, e.g.:
  // MyDynamicClass::init_type();
  OSSteerPlugIn::init_type();
  OSSteerVehicle::init_type();
  OSSteerManager::init_type();
  OSSteerPlugIn::register_with_read_factory();
  OSSteerVehicle::register_with_read_factory();

  return;
}

