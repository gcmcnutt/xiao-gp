# Use a ground TX controlled path selector for actual flights.

Here we allow the TX 6-position switch to select different pre-defined paths.

## Goals
- Ground controlled path selector
- switch is Sampled at the time of autoc=Y arming (where today's path generation) -- initially we can hard code this to channel 9 (similar to MSP_ARM_CHANNEL for the arming channel))
- then based on the switch postion one of 6 preprogrammed paths are selected
- this switch is a standard RC channel with values from 1000-2000 -- so the switch positions are evenly divided
- there are some differences in z offset for the paths in the training and in xiao flight -- so take that into consideration
- the path coordinates will be logged as today for POSTFLIGHT.md analysis
- we need to watch the path compute time -- hopefully it is << 100msec

## Method
- source code to change is in ~/xiao-gp/src
- source to inspect for how pathgen.h is used is in ~/GP/autoc
- use pathgen.h directl, we need to deprecate embedded pathgen at this point
- the differences between embedded_pathgen.h and pathgen.h may be mostly around runtime library requirements -- if this is too far so the result won't fit in the xiao, we may need to make a hybrid pathgen.h that has the needed functions but is more lightweight
- the path selector will choose from one of the aeroStandard paths (see GenerateAeroStandard and the pathIndex field -- we will generate an index based on 1000-2000 range of the selector channel)
- you will note that ~/GP/autoc is in the directory/search path of xiao-gp -- we use constructs like [#include <GP/autoc/gp_types.h>] to get to a peer file
  - if pathgen.h from ~/GP/autoc needs to be changed or subdivided we have several systems using it elsewhere, so some detailed analysis will be needed
- then when triggered, render the selected path
  - we will assume the craft is heading south as in the training runs at some safe elevation
  - and THAT should be the z offset for the pathgen synthesis (this it not quite like the training runs which start at z=-25)
  - this path can be cleared from memory when the autoc=N disarms via any method (switch, end of path, failsafe, etc)
- this is a platformio project, so generally we build the code with 'pio run' from the ~/xiao-gp directory

Key code to change is around msplink.cpp lines 377-385 -- here we see z capture and pathgen capture

So, examine the differences between embedded_pathgen.h and pathgen.h, understand how to merge this code.  And then understand how to render on the fly.  