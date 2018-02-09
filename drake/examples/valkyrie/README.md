# Valkyrie

This experimental directory does not adhere to the development practices that
apply elsewhere in Drake. In particular, for PRs that affect only this
directory, one feature review is sufficient; platform review is not required.

If build or test targets in this directory break, and the PR author or oncall
buildcop cannot trivially resolve them, a GitHub issue will be assigned to
the Valkyrie team. If the issue is not resolved within 24 hours, the author
or buildcop may disable the offending targets.


To run the visualizer:
  $ cd drake-distro
  $ ./build/install/bin/drake-visualizer

To run the pd + feedforward controller:
  $ cd drake-distro
  $ ./build/drake/examples/valkyrie/valkyrie_pd_ff_controller

To run the simulation:
  $ cd drake-distro
  $ ./build/drake/examples/valkyrie/valkyrie_simulation

The visualizer needs to be started before the simulator.
The controller and simulator are not synchronized in any way, and they both
try to run as fast as possible.
For a bit more repeatability, start the controller first.

The controlled simulation eventually fails when the robot falls down, because
the simple controller does not account for sliding feet.

# Historical note

Prior to 2016, Drake was built around a substantial base of MATLAB software.
Most of that was removed from the head of git master during 2017.

To view or use the original MATLAB implementation of Valkyrie you may use this
tag:

https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples/Valkyrie


# Documentation using Bazel - Ethan Weber
Make sure to compile everything with Gurobi support.
```
bazel build --config gurobi //...
```
Start the visualizer.
```
cd drake-distro
./bazel-bin/tools/drake_visualizer
```
In another terminal window, load the robot model. Note that it should load and fall to the ground in simulation without a controller. This means it is working!
```
cd drake-distro
./drake/examples/valkyrie/valkyrie_simulation
```
Now start the controller! I wrote a simplified controller that is flexible.
```
cd drake-distro/drake/examples/valkyrie
bazel run --config gurobi box
```
If this starts running correctly, restart the valkyrie simulation with the above command while the controller is running. This will ensure the controller starts with a valid input when the robot model loads.
