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

noinst_PROGRAMS = basic test1

BUILT_SOURCES = data.h

substDataDir = sed -e 's|@sampledatadir[@]|$(srcdir)/../data|g'

data.h: $(srcdir)/data.h.in Makefile
	$(substDataDir) $(srcdir)/$@.in > $@
	
EXTRA_DIST = data.h.in

common_sources = \
	$(srcdir)/../../source/config_module.cxx \
	$(srcdir)/../../source/opensteer_composite1.cxx \
	$(srcdir)/../../source/osSteerManager.cxx \
	$(srcdir)/../../source/osSteerPlugIn.cxx \
	$(srcdir)/../../source/osSteerVehicle.cxx \
	$(srcdir)/../../source/osTools.cxx \
	$(srcdir)/../../source/support/Draw.cpp \
	$(srcdir)/../../source/support/DrawMeshDrawer.cpp \
	$(srcdir)/../../source/support/SimpleVehicle.cpp

#basic
basic_SOURCES = \
	basic.cpp

nodist_basic_SOURCES = $(common_sources)

#test1
test1_SOURCES = \
	test1.cpp

nodist_test1_SOURCES = $(common_sources)

CLEANFILES = data.h