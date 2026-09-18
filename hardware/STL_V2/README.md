# V2 STL fabrication library

This folder contains the 31 root-level STL files supplied for the YO_ERROR-404 V2 robot. It intentionally contains no editable CAD source, drawing or image files.

## How to use the library

1. Start with a physical dry-fit of motor, bearings, differential, steering linkage, camera and sensor brackets.
2. Treat the different `V2`, `V3`, `V4` and `V5` labelled files as alternative iterations. Do not print every differential holder or coupler for one robot.
3. Record the chosen filename, filament, infill, nozzle size and test date in the engineering journal before a final print.
4. Use PETG for structural load-bearing mounts unless a part-specific test proves another material is suitable.

## File groups

| Group | STL files |
| --- | --- |
| Chassis and axle support | `aachassis body_6mm poly_1pc.stl`, `aapoly_6mm_1pc_front axel support.stl`, `aaside bearing mount_3mm AL_2pc.stl`, `front bearing_6_12_holder.STL`, `side bearing plate mount.STL` |
| Differential and couplers | `V2`, `V3`, `V4` and `V5` differential-holder files, differential couplers, motor coupling and back gear mount |
| Steering | `Turning FE V1.stl`, steering couplers and V2 steering coupler files |
| Wheels and camera | front/rear wheel files plus the camera-mount variants |
| Motor and electronics mounts | `V2 Motor mount for FE.stl`, `dc motor mount.STL`, `back rpi mount.STL`, `side mount tf mini mount.STL` |

The exact assembly mapping remains an open V2 build decision. Confirm fit against the actual purchased components before fabricating final parts.
