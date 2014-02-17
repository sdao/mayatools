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

SRCDIR := $(TOP)/ffdPlanarNode
DSTDIR := $(TOP)/ffdPlanarNode

ffdPlanarNode_SOURCES  := $(TOP)/ffdPlanarNode/ffdPlanarNode.cpp
ffdPlanarNode_OBJECTS  := $(TOP)/ffdPlanarNode/ffdPlanarNode.o
ffdPlanarNode_PLUGIN   := $(DSTDIR)/ffdPlanarNode.$(EXT)
ffdPlanarNode_MAKEFILE := $(DSTDIR)/Makefile

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

$(ffdPlanarNode_OBJECTS): CFLAGS   := $(CFLAGS)   $(ffdPlanarNode_EXTRA_CFLAGS)
$(ffdPlanarNode_OBJECTS): C++FLAGS := $(C++FLAGS) $(ffdPlanarNode_EXTRA_C++FLAGS)
$(ffdPlanarNode_OBJECTS): INCLUDES := $(INCLUDES) $(ffdPlanarNode_EXTRA_INCLUDES)

depend_ffdPlanarNode:     INCLUDES := $(INCLUDES) $(ffdPlanarNode_EXTRA_INCLUDES)

$(ffdPlanarNode_PLUGIN):  LFLAGS   := $(LFLAGS) $(ffdPlanarNode_EXTRA_LFLAGS) 
$(ffdPlanarNode_PLUGIN):  LIBS     := $(LIBS)   -lOpenMaya -lOpenMayaAnim -lFoundation $(ffdPlanarNode_EXTRA_LIBS) 

#
# Rules definitions
#

.PHONY: depend_ffdPlanarNode clean_ffdPlanarNode Clean_ffdPlanarNode


$(ffdPlanarNode_PLUGIN): $(ffdPlanarNode_OBJECTS) 
	-rm -f $@
	$(LD) -o $@ $(LFLAGS) $^ $(LIBS)

depend_ffdPlanarNode :
	makedepend $(INCLUDES) $(MDFLAGS) -f$(DSTDIR)/Makefile $(ffdPlanarNode_SOURCES)

clean_ffdPlanarNode:
	-rm -f $(ffdPlanarNode_OBJECTS)

Clean_ffdPlanarNode:
	-rm -f $(ffdPlanarNode_MAKEFILE).bak $(ffdPlanarNode_OBJECTS) $(ffdPlanarNode_PLUGIN)


plugins: $(ffdPlanarNode_PLUGIN)
depend:	 depend_ffdPlanarNode
clean:	 clean_ffdPlanarNode
Clean:	 Clean_ffdPlanarNode

