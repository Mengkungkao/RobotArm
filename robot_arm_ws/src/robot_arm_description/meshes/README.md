# Meshes

The model ships with primitive (cylinder) geometry only: it loads instantly,
Gazebo simulates it efficiently and it needs no external assets.

To use CAD meshes instead:

1. Put the visual meshes in `meshes/visual/` (`.dae` or `.stl`) and the
   simplified collision meshes in `meshes/collision/` (`.stl`, convex, low
   polygon count).
2. In `urdf/robot_arm_macro.xacro`, replace the `<geometry><cylinder .../></geometry>`
   block of the link you want to change with:

   ```xml
   <geometry>
     <mesh filename="package://robot_arm_description/meshes/visual/link_2.dae"/>
   </geometry>
   ```

3. Keep the **collision** geometry primitive (or a coarse convex hull) even
   when the visual geometry is a mesh - the planner and the physics engine both
   get much faster, and self-collision checking stays numerically stable.

Never change the link *frames* when swapping in meshes: the kinematics are
derived from `config/robot.yaml`, and a mesh whose origin does not match the
link frame will silently break both TF and the IK solution.
