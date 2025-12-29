# Postflight Analysis Directory

This directory contains flight logs and analysis from previous test flights.

## Important Note on Coordinate System Change (Dec 2024)

**The coordinate system was refactored to use canonical (0,0,0) origin for all path generation.**

Previous flight analysis documents (especially `flight24/PATH_GENERATOR_COMPARISON.md` and `flight24/GP_SENSOR_ANALYSIS.md`) reference the old coordinate system where:
- Paths were generated with `base = SIM_INITIAL_ALTITUDE` (-25m)
- Craft position had an additional `+= gp_vec3(0,0,SIM_INITIAL_ALTITUDE)` offset applied

**New coordinate system (current):**
- All paths generate at canonical origin (0,0,0)
- Craft position virtualized to (0,0,0) at arming via `test_origin_offset`
- Desktop simulation applies -25m offset in pathgen.cc after generation
- Embedded system uses paths directly at (0,0,0)

Future flight analysis should use the new coordinate system conventions.

## Flight Directories

- `bench14/` - Bench testing with simulator
- `flight21-24/` - Field test flights (coordinate system analysis)
- `blackbox-tools/` - Blackbox log decoder utilities
