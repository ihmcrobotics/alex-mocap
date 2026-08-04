# `alex-mocap`

Motion capture ground truth tracking package for the Alex humanoid robot. This package, as implemented, is currently only for the OptiTrack system at the IHMC Levin center, but the algorithm should be able to generalize to other motion capture systems/robots with some modifications.

## PRs and Milestones

<!-- make a table below for me to fill in, with |PR #| , |Description|, |Tests| -->
|PR #|Description|
|----|-----------|
|1|Everything that is robot independent|
|2|Plant and Recover|
|3|Runtime and Integration|

### PR 1
**Scope**: `core`, `registration`, `mocap`, and `gates`.

This should be everything that doesn't need to run on the robot, which includes the following tests: 
- Exact recovery (numbers should be robust for some random `Euclid.RigidBodyTransform`)
- Reflection Guard (against coplanar clusters that could be on the robot)
- Rank deficiency guard of the SVD for the rotation matrix (should be able to detect this structurally)
- Ensure that we can detect the MoCap error properly
- Implement G1 to ensure that a large drift that is not within Mocap error throws an error from this package. 
- **No garbage allocation** (this is a requirement for the robot, and for all the code written here, so we aren't populating the memory of the OCU/whatever runs this.)
- CSV round trip should export/import with no losses.

### PR 2
**Scope**: `model`, `frames`, `calibration`, and `gates`

### PR 3
**Scope**: `runtime`, `postprocess`, and `scs2`
