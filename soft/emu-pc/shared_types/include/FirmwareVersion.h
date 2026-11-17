#pragma once

/* The build passes the git revision it compiled as -DFW_VERSION and
 * -DFW_BUILD (see tools/gen_version.py). Host builds, the emulator and the
 * unit tests, have no definitions and report a development marker instead */
#ifndef FW_VERSION
#define FW_VERSION 0
#endif

#ifndef FW_BUILD
#define FW_BUILD "dev"
#endif
