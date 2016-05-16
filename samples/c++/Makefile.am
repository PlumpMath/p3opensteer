# recastnavigation tests
ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = \
	-I$(srcdir)/../../source \
	-I$(srcdir)/../../source/module \
	-I$(srcdir)/../../source/library \
	-I$(srcdir)/../../source/support \
	-I/usr/include/panda3d -I/usr/include/eigen3 \
	-DRN_DEBUG=

AM_CXXFLAGS = -O0 -g3 -Wall -Wno-reorder -fmessage-length=0 -std=c++11
AM_LDFLAGS = -L/usr/lib/x86_64-linux-gnu/panda3d

LDADD = -lp3framework -lpanda -lpandaexpress -lp3dtool -lp3dtoolconfig \
		-lp3pystub -lXxf86dga -lpython2.7 -lpandaphysics -lpthread

noinst_PROGRAMS = main test

BUILT_SOURCES = main.h

substDataDir = sed -e 's|@sampledatadir[@]|$(srcdir)/../data|g'

main.h: $(srcdir)/main.h.in Makefile
	$(substDataDir) $(srcdir)/$@.in > $@
	
EXTRA_DIST = main.h.in

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


#main
main_SOURCES = \
	main.cpp

nodist_main_SOURCES = $(common_sources)

#test
test_SOURCES = \
	test.cpp

nodist_test_SOURCES = $(common_sources)

CLEANFILES = main.h