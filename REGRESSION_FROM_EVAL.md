# Run an AUTOC evaluation across xiao-gp implementation to verify equivalence

In order to prove the xiao-gp controller can emit the same control outputs as ~/GP/autoc when it is using the ~/crsim/crrcsim-0.9.13 simulator (according to ~/GP/autoc/autoc-eval.ini), we want to develop a connector into xiao-gp that feeds ~/GP/autoc/data.dat content through the xiao-gp controller to verify that if the sensor input is the same. derived from data.dat, that the output will be close enough.

## Elements
The data.dat file emits one evalation per line. The source of the format of the data can be seen in ~/GP/autoc content.  Importantly, each line shows the index in the programmed flight path that was considered the 'goal' while the rest of the values are sensor inputs as visible by the GP nodes.

The outputs are roll/pitch/power.  The rest are diagnostic or sensor inputs.

Please check the autoc for details as to the GP nodes that are presently used and how they are derived so we can determine how to inject values from data.dat.

## steps
1. determine which flight path is computed in the autoc-eval.ini file.  render this as a virtual flight, the same as autoc does.
2. for each line of data.dat, read the sensor inputs and the path's goal index.
3. set the virtual flight state to what is seen in the goal index. (keep in mind that the GP needs to know this location as it has lookahead capabilities based on forecast path).
4. make the sensor inputs available to the evaluator portion of the xiao-gp controller. note, today this is tightly integrated with the msp timing loop.  for this exercise we are mostly verifying the raw sensors to the integrated node values which we will pass through the evaluator.
5. capture or log the roll/pitch/power outputs from the xiao-gp controller.
6. compare the outputs to the data.dat outputs.

## key considerations
1. we will want to do this multiple times over improvements so, this is sort of a special compilation that will run on actual hardware, probably a flag or conditional compile for now. another possibility is as a platformio unit test that can run on the different cpu architecture of the pio build host.
2. if this is done as a unit test, we can open data.dat file as a command argument. if we decide to verify this on flight hardware, we probably need to compile in the data.dat and perhaps synthesized path.
3. we will capture results from the standard logger in place, no need for additional complexity.
4. we can have all the rest of the state machine in place so that when we arm autoc it starts its simulation based on data.dat simulation times, not on wall clock.  depends on if this is best as a unit test or on flight hardware.

## native platformio unit test experiment
* add a new `env:native_eval` to `platformio.ini` so we can compile a replay harness against the host toolchain and run it under `pio test`.
* share the replay logic between host and embedded builds via a small helper (e.g. `lib/autoc_test_helpers`); guard file IO and other desktop-only pieces with `#if !defined(ARDUINO)`.
* read `GP/autoc/data.dat` directly when running on the host to construct the path vector and per-step sensor snapshots, then feed those into the existing `generatedGPProgram` entry point.
* track any `#ifdef` churn while wiring this up—if the native path becomes messy, pivot back to embedding the dataset and running the suite only on-flight hardware.

## native prototype results (2024-11-16)
* `pio test -e native_eval` now builds a Unity harness that replays 64 samples straight from `GP/autoc/data.dat` using the shared `lib/autoc_test_helpers` helpers; the only new conditional code is the `USE_NATIVE_EVAL` gate that pulls the generated GP bytecode and portable evaluator into the host-only build.
* Current run fails on roll by ~0.053 at sample 38 (RMSE roll/pitch/throttle ≈ 0.0099 / 0.0055 / 0.0027). This gives us a concrete delta to chase without flashing hardware.
* Next steps: tighten the replay fidelity (likely quaternion/velocity conversions) or relax the tolerance once we've compared the raw MSP logs to AUTOC's simulated values. Once the comparison is within tolerance we can promote the test to CI and consider a matching embedded harness.
