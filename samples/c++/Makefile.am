# recastnavigation tests
ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = \
	-I$(srcdir)/../../source \
	-I$(srcdir)/../../source/library \
	-I$(srcdir)/../../source/support \
	-I/usr/include/panda3d -I/usr/include/eigen3 \
	-DOS_DEBUG=

AM_CXXFLAGS = -O0 -g3 -Wall -Wno-reorder -fmessage-length=0 -std=c++11
AM_LDFLAGS = -L/usr/lib/x86_64-linux-gnu/panda3d

LDADD = -lp3framework -lpanda -lpandaexpress -lp3dtool -lp3dtoolconfig \
		-lp3pystub -lXxf86dga -lpython2.7 -lpandaphysics -lpthread

noinst_HEADERS = common.h

noinst_PROGRAMS = basic low_speed_turn

BUILT_SOURCES = data.h

substDataDir = sed -e 's|@sampledatadir[@]|$(srcdir)/../data|g'

data.h: $(srcdir)/data.h.in Makefile
	$(substDataDir) $(srcdir)/$@.in > $@
	
EXTRA_DIST = data.h.in

common_sources = \
	common.cpp \
	$(srcdir)/../../source/osSteerManager.cxx \
	$(srcdir)/../../source/osSteerPlugIn.cxx \
	$(srcdir)/../../source/osSteerVehicle.cxx \
	$(srcdir)/../../source/osTools.cxx \
	$(srcdir)/../../source/library/OpenSteer/Camera.cpp \
	$(srcdir)/../../source/library/OpenSteer/Clock.cpp \
	$(srcdir)/../../source/library/OpenSteer/Color.cpp \
	$(srcdir)/../../source/library/OpenSteer/lq.c \
	$(srcdir)/../../source/library/OpenSteer/Obstacle.cpp \
	$(srcdir)/../../source/library/OpenSteer/OldPathway.cpp \
	$(srcdir)/../../source/library/OpenSteer/Path.cpp \
	$(srcdir)/../../source/library/OpenSteer/Pathway.cpp \
	$(srcdir)/../../source/library/OpenSteer/PlugIn.cpp \
	$(srcdir)/../../source/library/OpenSteer/PolylineSegmentedPath.cpp \
	$(srcdir)/../../source/library/OpenSteer/PolylineSegmentedPathwaySegmentRadii.cpp \
	$(srcdir)/../../source/library/OpenSteer/PolylineSegmentedPathwaySingleRadius.cpp \
	$(srcdir)/../../source/library/OpenSteer/SegmentedPath.cpp \
	$(srcdir)/../../source/library/OpenSteer/SegmentedPathway.cpp \
	$(srcdir)/../../source/library/OpenSteer/TerrainRayTest.cpp \
	$(srcdir)/../../source/library/OpenSteer/Vec3.cpp \
	$(srcdir)/../../source/library/OpenSteer/Vec3Utilities.cpp \
	$(srcdir)/../../source/support/Draw.cpp \
	$(srcdir)/../../source/support/DrawMeshDrawer.cpp \
	$(srcdir)/../../source/support/SimpleVehicle.cpp
#	$(srcdir)/../../source/library/OpenSteer/main.cpp
#	$(srcdir)/../../source/library/OpenSteer/OpenSteerDemo.cpp
#	$(srcdir)/../../source/library/OpenSteer/Draw.cpp
#	$(srcdir)/../../source/library/OpenSteer/SimpleVehicle.cpp

#basic
basic_SOURCES = \
	basic.cpp

nodist_basic_SOURCES = $(common_sources)

#low_speed_turn
low_speed_turn_SOURCES = \
	low_speed_turn.cpp

nodist_low_speed_turn_SOURCES = $(common_sources)

CLEANFILES = data.h