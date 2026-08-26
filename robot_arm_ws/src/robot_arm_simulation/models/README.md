# Gazebo models

This directory is exported as the package's `gazebo_model_path`, so anything
placed here is found by Gazebo without extra environment variables.

Put static scene objects here - a table, a fixture, a bin - as regular Gazebo
model directories (`<model_name>/model.config` + `<model_name>/model.sdf`), then
include them from `worlds/robot_arm.world`:

```xml
<include>
  <uri>model://my_table</uri>
  <pose>0.5 0 0 0 0 0</pose>
</include>
```

The robot itself does not live here: it is spawned from the URDF that
`robot_arm_description` generates, so simulation and hardware always share one
description.
