#!/usr/bin/env bash
# Re-fetch every upstream repository the design is built on (shallow clones).
set -euo pipefail
cd "$(dirname "$0")/../reference"
clone() { [[ -d "$2" ]] || git clone --depth 1 ${3:+--branch "$3"} "$1" "$2"; }
clone https://github.com/pollen-robotics/microduck_rl.git          microduck_rl            # MJCF + 43 STL meshes (CC BY-NC-SA)
clone https://github.com/pollen-robotics/microduck.git             microduck               # Rust runtime, 9 ONNX policies (Apache-2.0)
clone https://github.com/apirrone/Open_Duck_Mini.git               Open_Duck_Mini v2       # open hardware duck: 129 print STLs, BOM (Apache-2.0)
clone https://github.com/apirrone/Open_Duck_Playground.git         Open_Duck_Playground    # ODM v2 MJCF, STS3215 actuator model, training
clone https://github.com/apirrone/Open_Duck_Mini_Runtime.git       Open_Duck_Mini_Runtime  # Pi runtime: obs layout, init pose, gains
clone https://github.com/fanhao375/microduck-replica.git           microduck-replica       # electronics teardown, fastener study, per-assembly STLs
clone https://github.com/lingzolabs/microduck-hardware-replica.git microduck-hardware-replica  # 70 world-positioned STLs, FreeCAD assembly
clone https://github.com/boris721/microduck-3d.git                 microduck-3d            # combined GLB + kinematics.json
mkdir -p press && cd press
for n in desk skate walkabout watching; do
  [[ -f microduck-$n.jpg ]] || curl -sL -o microduck-$n.jpg "https://pollen-robotics.com/assets/microduck/press/photos/microduck-$n.jpg"
done
echo done
