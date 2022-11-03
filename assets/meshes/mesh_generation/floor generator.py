import pygmsh
import numpy as np
import os

floorWidth = 500
floorLength = 150
floorThickness = 2
floorStart = [-0.5*floorWidth,-10,0]
floor = [500,150,2]

def make_box(geom, dims, start, mesh_size=4.0):
    dims = np.asarray(dims)
    start = np.asarray(start)
    points = [
        start,
        start + dims * np.array([1, 0, 0]),
        start + dims * np.array([1, 1, 0]),
        start + dims * np.array([0, 1, 0]),
    ]
    poly = geom.add_polygon(points, mesh_size=mesh_size)
    return geom.extrude(poly, [0, 0, dims[2]])[1]

def make_collision_mesh(geom):
    parts = []
    parts.append(make_box(geom, floor, floorStart, 10.0))
    return geom.boolean_union(parts)[0]

def generate(outdir, translate=(0, 0, 0), angle=0., axis=(0, 0, 1.0)):
    os.makedirs(outdir, exist_ok=True)
    with pygmsh.occ.Geometry() as geom:
        collision = make_collision_mesh(geom)
        geom.translate(collision, translate)
        geom.rotate(collision, (0, 0, 0), angle, axis)
        mesh = geom.generate_mesh(dim=2)
        mesh.write(os.path.join(outdir, "floor.stl"))

if __name__ == '__main__':
    generate('./floor/', (0, 0, 0), 0., (0, 0, 1))
