#-
# ==========================================================================
# Copyright (c) 2011 Autodesk, Inc.
# All rights reserved.
# 
# These coded instructions, statements, and computer programs contain
# unpublished proprietary information written by Autodesk, Inc., and are
# protected by Federal copyright law. They may not be disclosed to third
# parties or copied or duplicated in any form, in whole or in part, without
# the prior written consent of Autodesk, Inc.
# ==========================================================================
#+

ifndef INCL_BUILDRULES

#
# Include platform specific build settings
#
TOP := ..
include $(TOP)/buildrules


#
# Always build the local plug-in when make is invoked from the
# directory.
#
all : plugins

endif

#
# Variable definitions
#

SRCDIR := $(TOP)/ffdOneNode
DSTDIR := $(TOP)/ffdOneNode

ffdOneNode_SOURCES  := $(TOP)/ffdOneNode/ffdOneNode.cpp
ffdOneNode_OBJECTS  := $(TOP)/ffdOneNode/ffdOneNode.o
ffdOneNode_PLUGIN   := $(DSTDIR)/ffdOneNode.$(EXT)
ffdOneNode_MAKEFILE := $(DSTDIR)/Makefile

#
# Include the optional per-plugin Makefile.inc
#
#    The file can contain macro definitions such as:
#       {pluginName}_EXTRA_CFLAGS
#       {pluginName}_EXTRA_C++FLAGS
#       {pluginName}_EXTRA_INCLUDES
#       {pluginName}_EXTRA_LIBS
-include $(SRCDIR)/Makefile.inc


#
# Set target specific flags.
#

$(ffdOneNode_OBJECTS): CFLAGS   := $(CFLAGS)   $(ffdOneNode_EXTRA_CFLAGS)
$(ffdOneNode_OBJECTS): C++FLAGS := $(C++FLAGS) $(ffdOneNode_EXTRA_C++FLAGS)
$(ffdOneNode_OBJECTS): INCLUDES := $(INCLUDES) $(ffdOneNode_EXTRA_INCLUDES)

depend_ffdOneNode:     INCLUDES := $(INCLUDES) $(ffdOneNode_EXTRA_INCLUDES)

$(ffdOneNode_PLUGIN):  LFLAGS   := $(LFLAGS) $(ffdOneNode_EXTRA_LFLAGS) 
$(ffdOneNode_PLUGIN):  LIBS     := $(LIBS)   -lOpenMaya -lOpenMayaAnim -lFoundation $(ffdOneNode_EXTRA_LIBS) 

#
# Rules definitions
#

.PHONY: depend_ffdOneNode clean_ffdOneNode Clean_ffdOneNode


$(ffdOneNode_PLUGIN): $(ffdOneNode_OBJECTS) 
	-rm -f $@
	$(LD) -o $@ $(LFLAGS) $^ $(LIBS)

depend_ffdOneNode :
	makedepend $(INCLUDES) $(MDFLAGS) -f$(DSTDIR)/Makefile $(ffdOneNode_SOURCES)

clean_ffdOneNode:
	-rm -f $(ffdOneNode_OBJECTS)

Clean_ffdOneNode:
	-rm -f $(ffdOneNode_MAKEFILE).bak $(ffdOneNode_OBJECTS) $(ffdOneNode_PLUGIN)


plugins: $(ffdOneNode_PLUGIN)
depend:	 depend_ffdOneNode
clean:	 clean_ffdOneNode
Clean:	 Clean_ffdOneNode

