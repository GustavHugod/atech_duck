#!/usr/bin/env python3
"""Render the two reference ducks and the Atech 14-port board at one scale.

    python scripts/render_reference.py            -> docs/img/reference_lineup.png
Needs out/microduck.stl and out/open_duck_mini_v2.stl from assemble_mjcf.py.
"""
import os, sys, numpy as np, pyvista as pv
pv.OFF_SCREEN = True
ATECH = os.path.expanduser('~/atech_code/public/modules/3d/motherboard_14_port.stl')

def load_zup(path, yup=False):
    m = pv.read(path)
    if yup:  # Atech STLs are Y-up: (x, y, z) -> (x, -z, y)
        m = m.rotate_x(90, inplace=False)
    b = m.bounds
    m = m.translate((-(b[0]+b[1])/2, -(b[2]+b[3])/2, -b[4]), inplace=False)  # centre xy, feet on z=0
    return m

md = load_zup('out/microduck.stl')
od = load_zup('out/open_duck_mini_v2.stl')
bd = load_zup(ATECH, yup=True)
# the board is flat on the floor in front, plus a standing copy so the height reads
bd_flat = bd.copy()
bd_std = bd.rotate_x(90, inplace=False); b = bd_std.bounds; bd_std = bd_std.translate((0, 0, -b[4]), inplace=False)

pl = pv.Plotter(window_size=(1800, 1000))
pl.set_background('white')
pl.add_mesh(md.translate((0, -260, 0), inplace=False), color=(0.96, 0.90, 0.72), smooth_shading=True, specular=0.2)
pl.add_mesh(od.translate((0, 0, 0), inplace=False), color=(0.82, 0.84, 0.88), smooth_shading=True, specular=0.2)
pl.add_mesh(bd_flat.translate((0, 260, 0), inplace=False), color=(0.16, 0.45, 0.30), smooth_shading=True)
pl.add_mesh(bd_std.translate((-10, 340, 0), inplace=False), color=(0.16, 0.45, 0.30), smooth_shading=True)
floor = pv.Plane(center=(0, 40, -0.5), direction=(0, 0, 1), i_size=700, j_size=900)
pl.add_mesh(floor, color=(0.93, 0.93, 0.93))
# 100 mm scale bar
bar = pv.Line((-160, 420, 0.5), (-60, 420, 0.5)); pl.add_mesh(bar, color='black', line_width=6)
labels = [((0, -260, 300), 'Microduck  272 mm  737 g\n15x XL330'),
          ((0, 0, 520), 'Open Duck Mini v2  489 mm  2107 g\n14x STS3215'),
          ((0, 300, 150), 'Atech 14-port board\n60 x 120 x 7 mm'),
          ((-110, 420, 12), "100 mm")]
pl.add_point_labels([l[0] for l in labels], [l[1] for l in labels], font_size=22, point_size=0, shape=None, text_color='black', always_visible=True)
pl.camera_position = [(-1250, -750, 620), (0, 30, 210), (0, 0, 1)]
pl.add_light(pv.Light(position=(-600, -800, 900), intensity=0.8))
os.makedirs('docs/img', exist_ok=True)
pl.screenshot('docs/img/reference_lineup.png')
print('wrote docs/img/reference_lineup.png')
