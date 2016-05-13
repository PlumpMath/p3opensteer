#ifndef CONFIG_OPENSTEER_H
#define CONFIG_OPENSTEER_H

#pragma once

#include "pandabase.h"
#include "notifyCategoryProxy.h"
#include "configVariableDouble.h"
#include "configVariableString.h"
#include "configVariableInt.h"


NotifyCategoryDecl(opensteer, EXPORT_CLASS, EXPORT_TEMPL);

extern void init_libopensteer();

#endif
