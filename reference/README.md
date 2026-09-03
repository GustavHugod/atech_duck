# Reference material (fetched 2026-09-02, `scripts/fetch_reference.sh` re-creates it)

There is no 3D *scan* of the Microduck anywhere public, and Pollen has not
released the CAD. What exists, and what each folder is:

| folder | what | licence | use here |
|---|---|---|---|
| `microduck_rl/src/mjlab_microduck/robot/microduck/` | **The official meshes.** 43 STL files (all 47 parts incl. XL330 servo, NP-F battery, PCBs, bearings) + MJCF with every joint origin, axis, range, mass and inertia. Exported from Pollen's Onshape with onshape-to-robot. | CC BY-NC-SA 4.0 (meshes), Apache-2.0 (code) | `scripts/assemble_mjcf.py` poses them → `out/microduck.{stl,glb}`, `docs/anatomy_microduck.md` |
| `microduck/` | Rust runtime + the 9 shipped ONNX policies (`policies/`), docs, `kinematics/assets/alpha/robot_walk.xml` | Apache-2.0 | policy structure (61→512→256→128→14 ELU), servo bus facts |
| `microduck-replica/` | Community reverse-engineering: electronics teardown from the source (Radxa Zero 3W, XL330 bus, IMU-on-bus, HAT), fastener study (M2 system), 15 per-assembly STLs, actuator comparison XL330 vs STS3215 | mixed | `docs/hardware-teardown.en.md`, `docs/actuator-selection.en.md` |
| `microduck-hardware-replica/` | 70 STL occurrences already placed in STAND-pose world coordinates (mm), FreeCAD multipart assembly, planning BOM (xlsx) | CC BY-NC-SA | `hardware/meshes/parts_world/` |
| `microduck-3d/` | Same meshes + a combined GLB and `kinematics.json` (joint tree as JSON) | CC BY-NC-SA | quick viewer |
| `press/` | Pollen press photos (desk, skate, walkabout, watching) | press use | look-and-feel reference |
| `Open_Duck_Mini/` | **Open Duck Mini v2** — the open-hardware predecessor by the same engineer: 129 print STLs (`print/`), Onshape link, BOM, assembly guide, wiring, Feetech STS3215 servos | Apache-2.0 | the body this project builds on |
| `Open_Duck_Playground/` | ODM v2 MJCF (`playground/open_duck_mini_v2/xmls/`) with the calibrated STS3215 actuator model and the RL task | Apache-2.0 | `out/open_duck_mini_v2.{stl,glb}`, retraining |
| `Open_Duck_Mini_Runtime/` | ODM v2 Raspberry Pi runtime: 101-D observation, init pose, gains, bus IDs, `BEST_WALK_ONNX_2.onnx` | Apache-2.0 | firmware contract (`firmware/src/duck_config.h`) |

Onshape (view only, no export without an account):
- Microduck: https://cad.onshape.com/documents/804927696f06d877f3f1803e/w/5b75db19292e71970de02dee/e/ef6e972847fec8d82570b35e
- Open Duck Mini v2: https://cad.onshape.com/documents/64074dfcfa379b37d8a47762/w/3650ab4221e215a4f65eb7fe/e/0505c262d882183a25049d05
