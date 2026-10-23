#!/usr/bin/env python3
"""Generate the convex unit sphere scaled into a rounded tire collision mesh.

The rounded tread keeps the ground contact near the wheel centreline. A flat
cylinder can contact at its inner rim in DART, changing the effective track.
Visual tires and the configured physical radius/width remain unchanged.
"""
import math
from pathlib import Path
import struct


def point(lat, lon):
    return (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))


def main():
    triangles = []
    levels, segments = 24, 64
    for i in range(levels):
        a, b = -math.pi/2 + i*math.pi/levels, -math.pi/2 + (i+1)*math.pi/levels
        for j in range(segments):
            c, d = j*2*math.pi/segments, (j+1)*2*math.pi/segments
            p, q, r, s = point(a, c), point(a, d), point(b, d), point(b, c)
            if i:
                triangles.append((p, q, r))
            if i+1 < levels:
                triangles.append((p, r, s))
    path = Path(__file__).resolve().parents[1] / 'src/smartcar_description/meshes/unit_sphere.stl'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as f:
        f.write(b'Rounded tire collision: unit sphere'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            f.write(struct.pack('<12fH', 0., 0., 0., *(v for p in triangle for v in p), 0))
    print(path)


if __name__ == '__main__':
    main()
