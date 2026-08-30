# Meshes

The arm ships with primitive geometry only - boxes and cylinders composed into
a cast base, a rotating column, a tapered upper arm, a tubular forearm and a
three-roll wrist. It loads instantly, Gazebo simulates it efficiently, MoveIt
collision-checks it cheaply, and it needs no external assets.

## Swapping in CAD meshes

1. Put the visual meshes in `meshes/visual/` (`.dae` or `.stl`) and simplified
   collision meshes in `meshes/collision/` (`.stl`, convex, low polygon count).

2. Each link is assembled in `urdf/robot_arm_macro.xacro` from `cyl_part` and
   `box_part` calls. Replace the parts of the link you are re-skinning with a
   `<visual>` that points at your mesh:

   ```xml
   <visual>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <mesh filename="package://robot_arm_description/meshes/visual/link_2.dae"/>
     </geometry>
     <material name="arm_orange"/>
   </visual>
   ```

3. Keep the **collision** geometry primitive, even when the visual geometry is
   a mesh. The planner and the physics engine both get much faster, and
   self-collision checking stays numerically stable. `cyl_part` and `box_part`
   take `collision:=false` if you want a visual-only part.

4. Re-run the collision audit afterwards - a new shape can bring links together
   that could not touch before:

   ```bash
   xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
   ros2 run robot_arm_moveit_config collision_audit.py /tmp/arm.urdf \
       ../robot_arm_moveit_config/config/robot_arm.srdf
   ```

## What you must not change

Never move the link *frames* when swapping in meshes. The kinematics are
derived from the `kinematics:` block of `config/robot.yaml`, and a mesh whose
origin does not match its link frame breaks TF and every IK solution while
still looking plausible on screen.

Check the result without RViz:

```bash
ros2 run robot_arm_description urdf_preview.py /tmp/arm.urdf /tmp/arm.png 0,40,-60,0,45,0
```
