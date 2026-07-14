# Bipedal Robot Specification

This document records the currently known physical dimensions, mass breakdown, and weight distribution of the self-balancing bipedal robot.

Measurement priority used here:

1. Hand-measured values from the user take priority.
2. Repo values are used when the user has not given a number.
3. CAD is treated as a secondary cross-check only, not the primary source.

## 1. Robot Summary

- Robot type: self-balancing bipedal robot with two wheel drive modules and two articulated legs.
- Leg actuators: 4 x AX-12+ servos.
- Wheel motors: 2 x JGA12V 300 rpm motors.
- Power source: 1 x 3300 mAh 3S LiPo battery mounted below the servo box, low in the chassis.
- Main structure: 3D-printed body shell/frame with the 4 servos inside the box and the legs attached outside.

## 1.1 Mechanical Architecture

- The wheel motors and the leg servos are separate subsystems.
- The JGA12V motors drive the wheels and are responsible for the base motion and balance correction.
- The AX-12+ servos drive the leg linkages inside the servo box.
- The leg geometry used by the digital twin is a calibrated kinematic model of the real linkage layout.

## 2. Known Dimensions

### Overall and body dimensions

- Main body width between the lower joints: 147 mm.
- Out-to-out width across the wheels/shaft/motor assembly: 270 mm.
- Front-to-back body length: 117 mm.
- Front-to-back outer span from the later hand note: about 120 mm.
- Actual servo box length: 148 mm.
- Actual servo box width: 118 mm.
- Actual servo box height: 89 mm.
- Battery-included box height: 113 mm.
- Full assembled height: just over 13 in, approximately 330 mm.

Secondary cross-check note:

- The 148 x 118 x 89 mm box is the hand-measured robot box used as the primary reference in this file.
- The smaller 140 x 80 x 80 mm demo box was mentioned separately but is not treated as the main robot geometry.

### Placement heights

- Servo location height from ground: about 8 in, approximately 203 mm.
- Battery top height from ground: about 7 in, approximately 178 mm.
- Wheel center height from ground: less than 2 in, approximately under 51 mm.
- Battery sits below the box, under the servos.

## 2.2 Model-Building Interpretation

If another AI coding agent uses this document to build a robot model, it should treat the geometry as follows:

- The servo box dimensions are the primary chassis reference.
- The linkage model is defined by the servo mount spacing, femur length, tibia length, and leg separation.
- The foot target is the IK input; the knee coordinates are the solved internal geometry.
- The wheel motors, battery, and servo box are physically distinct components, even though they share the same robot body.
- CAD should be used only to fill in shell and bracket shape, not to replace the hand-measured kinematic constants above.

### Leg spacing

- Distance between the two legs in the digital twin and leg-control tools: 180 mm.

## 2.1 Joints and Linkages

### Servo IDs and leg pairing

- Leg 1 left servo ID: 6.
- Leg 1 right servo ID: 14.
- Leg 2 left servo ID: 0.
- Leg 2 right servo ID: 1.

### Per-leg joint geometry

- Left servo mount point in the local leg frame: x = -30 mm, y = 0 mm.
- Right servo mount point in the local leg frame: x = +30 mm, y = 0 mm.
- Femur length: 55 mm.
- Tibia length: 100 mm.
- The two-leg X offset used by the solver: 180 mm.

### How the linkage is connected

1. The foot target `(x, y)` is the commanded end point for a leg.
2. The solver places two fixed servo mounts inside the box at `(-30, 0)` and `(30, 0)`.
3. Each servo drives an upper link of 55 mm.
4. The upper link ends are connected to the lower link through a knee intersection point.
5. The lower link is 100 mm long and connects the knee point to the foot target.
6. The real servo commands are computed from the joint angles, then mapped to AX-12 goal positions.

### Knee selection rule

- For Leg 1, the left knee uses the intersection with the smaller X value.
- For Leg 1, the right knee uses the intersection with the larger X value.
- This gives the outward-bending knee geometry used in the twin.

### Leg 2 mirroring

- Leg 2 is treated as a mirrored mount.
- With mirrored mounting enabled, the solver swaps and negates the computed angles before mapping them to AX-12 positions.
- The leg-2 angle order becomes `-Angle_R` for the left servo and `-Angle_L` for the right servo.

### AX-12 straight-down calibration

- Straight-down standing pose is calibrated to 818 for left servos and 441 for right servos.
- The servo mapping is calibrated, so the model should not assume a generic 150-degree center.
- This offset is part of the physical robot and not just a simulation detail.

### Kinematic role of the twin files

- `twin_kinematics.py` is the shared source of truth for the tuned femur/tibia geometry and servo mapping.
- `leg_control.py` uses the same solver to move the real servos by foot position or angle.
- `digital_twin_legs.py` uses the same solver to visualize the linkage and send positions live.

### Minimum model details needed for reconstruction

To rebuild the robot mechanism from this file, an AI model should infer:

- A central servo box with four AX-12 servos mounted inside it.
- Two mirrored leg modules mounted outside the box.
- Two-link leg chains per side with 55 mm upper links and 100 mm lower links.
- A 180 mm separation between the two leg assemblies.
- A battery mounted below the servo box.
- Wheel motors separate from the linkage and mounted low in the chassis.

### Wheel size

- Wheel diameter: 65 mm.

## 3. Mass Breakdown

Known component masses from the user measurements:

- 3D-printed body: about 300 g.
- 3S LiPo battery: about 260 g.
- AX-12+ servos: 53.5 g each, 4 total, about 214 g combined.
- JGA12V motors: 150 g each, 2 total, about 300 g combined.

Known subtotal from the items above:

- Approximate subtotal: 1074 g.

Still unconfirmed in the notes provided:

- Electronics mass, including STM32 board, wiring, motor driver, IMU, regulators, and connectors.
- Any fasteners, brackets, shafts, wheel hubs, bearings, and miscellaneous hardware.

## 4. Weight Distribution

### Main mass locations

- Battery: mounted below the servo box, low in the chassis.
- Servos: inside the box, above the battery.
- Wheel motors: low and to the sides near the wheel assembly.
- 3D-printed body: distributed through the central frame and box.

### Relative weightage

- Heaviest single listed items: battery and the two motors.
- Medium contributor: the 3D-printed body.
- Smaller contributors: the four servos individually, although together they are still significant.

### Balance effect

- The battery placement lowers the center of mass.
- The motors add low-side lateral mass near the wheel line.
- The servos sit above the battery, so they raise the upper mass slightly but remain inside the box.
- This layout is consistent with a low-slung balancing robot where the heaviest parts are kept close to the ground.

### Still missing for a full CAD-style build

- Exact bracket thicknesses and hole locations for the servo box.
- Exact wheel motor mount geometry.
- Exact linkage rod diameters and end-joint hardware dimensions.
- Exact wheel hub and axle details.
- Exact total mass and center of mass.

## 5. Notes From Existing Repo Files

- The tuned leg spacing used in the digital twin and leg-control scripts is 180 mm.
- The robot uses AX-12+ servos and 11.1V to 12V power for the servo bus.
- The firmware and twin files in the repo assume the AX-12+ leg geometry and ID layout already documented in the code.

## 6. Open Items

These values are still missing if you want a fully exact spec sheet:

- Exact measured total robot mass.
- Exact electronics mass.
- Exact vertical coordinates of each major subassembly if you want a formal center-of-mass estimate.
- Exact wheelbase / axle separation if you want that recorded separately from wheel diameter.
- If you want the 120 mm front-to-back number used instead of 117 mm, I can swap it in as the headline dimension and keep 117 mm as the alternate hand note.

## 7. CAD Export Recommendation

If you attach the CAD model, the preferred Fusion export format is STEP (.step or .stp). Native Fusion .f3d is also acceptable if STEP is not convenient.