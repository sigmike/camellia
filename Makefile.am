AUTOMAKE_OPTIONS = gnu
SUBDIRS = src

EXTRA_DIST = src/*_code.c src/*_table*.c src/*_code_*.c
bin_PROGRAMS = cam_demo cam_yalefaces cam_myobjects
cam_demo_SOURCES = cam_demo.c cam_demo_cpp.cpp
cam_demo_LDADD = src/libCamellia.la
cam_demo_LDFLAGS = -no-install

cam_yalefaces_SOURCES = cam_yalefaces.c
cam_yalefaces_LDADD = src/libCamellia.la
cam_yalefaces_LDFLAGS = -no-install

cam_myobjects_SOURCES = cam_myobjects.c
cam_myobjects_LDADD = src/libCamellia.la
cam_myobjects_LDFLAGS = -no-install

