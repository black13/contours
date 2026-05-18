"""Polytope primitives with known equations — no camera culling, no subdivision artifacts."""

import math

Vec = tuple[float, float, float]
Face = tuple[int, ...]


def icosahedron() -> tuple[list[Vec], list[Face]]:
    """Regular icosahedron centered at origin. Edge length = 2.
    
    Equation: vertices are permutations of (0, ±1, ±φ) scaled to unit sphere.
    φ = golden ratio. All vertices lie on sphere of radius √(1+φ²).
    """
    phi = (1.0 + math.sqrt(5.0)) * 0.5
    verts = [
        ( 0,  1,  phi), ( 0, -1,  phi), ( 0,  1, -phi), ( 0, -1, -phi),
        ( 1,  phi, 0), (-1,  phi, 0), ( 1, -phi, 0), (-1, -phi, 0),
        ( phi, 0,  1), (-phi, 0,  1), ( phi, 0, -1), (-phi, 0, -1),
    ]
    faces = [
        (0,1,4), (0,4,5), (0,5,8), (0,8,9), (0,9,1),
        (1,9,7), (1,7,6), (1,6,4),
        (2,3,6), (2,6,7), (2,7,11),(2,11,10),(2,10,3),
        (3,10,4),(3,4,6),
        (5,4,10),(5,10,8),
        (8,10,11),(8,11,9),
        (9,11,7)
    ]
    return verts, faces


def cube() -> tuple[list[Vec], list[Face]]:
    """Unit cube centered at origin. Edge length = 2.
    
    Vertices: (±1, ±1, ±1). All lie on sphere of radius √3.
    """
    verts = [
        (-1,-1,-1), ( 1,-1,-1), ( 1, 1,-1), (-1, 1,-1),
        (-1,-1, 1), ( 1,-1, 1), ( 1, 1, 1), (-1, 1, 1),
    ]
    faces = [
        (0,3,2,1), (4,5,6,7), (0,1,5,4),
        (2,3,7,6), (0,4,7,3), (1,2,6,5),
    ]
    return verts, faces


def tetrahedron() -> tuple[list[Vec], list[Face]]:
    """Regular tetrahedron centered at origin.
    
    Vertices: (1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1).
    Circumscribed sphere radius = √3.
    """
    verts = [(1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1)]
    faces = [(0,2,1), (0,1,3), (0,3,2), (1,2,3)]
    return verts, faces


def cylinder(radius: float = 1.0, height: float = 2.0, segments: int = 32) -> tuple[list[Vec], list[Face]]:
    """Cylinder of given radius and height, centered at origin.
    
    Equation: x² + y² = R², -H/2 ≤ z ≤ H/2.
    Not a polytope — a quad mesh approximation.
    """
    h = height * 0.5
    verts = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        verts.append((radius * math.cos(a), radius * math.sin(a), -h))
        verts.append((radius * math.cos(a), radius * math.sin(a),  h))
    
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        faces.append((
            i * 2, j * 2, j * 2 + 1, i * 2 + 1
        ))
    return verts, faces


def subdivide_catmull_clark(
    verts: list[Vec], faces: list[Face], levels: int = 1
) -> tuple[list[Vec], list[Face]]:
    """Catmull-Clark subdivision — makes polytope approach sphere.
    
    For each quad face:
      - Add face point (average of face vertices)
      - Add edge point (average of edge endpoints + adjacent face points)
      - Move original vertex toward weighted average
    Returns subdivided mesh.
    """
    if levels <= 0:
        return verts, faces
    
    # This is a simplified Catmull-Clark — for the icosahedron (all triangles),
    # we just do one level of Loop subdivision or convert to quads first.
    # For simplicity: use trimesh for subdivision.
    import trimesh
    tm = trimesh.Trimesh(vertices=verts, faces=faces)
    for _ in range(levels):
        tm = tm.subdivide()
    v = [(float(p[0]), float(p[1]), float(p[2])) for p in tm.vertices]
    f = [tuple(int(i) for i in face) for face in tm.faces]
    return v, f


def sphere(subdiv: int = 3) -> tuple[list[Vec], list[Face]]:
    """Approximate sphere via subdivided icosahedron.
    
    Icosahedron → subdivide subdiv times → project to unit sphere.
    All vertices lie exactly on sphere of radius 1.0.
    """
    verts, faces = icosahedron()
    for _ in range(subdiv):
        verts, faces = subdivide_catmull_clark(verts, faces, 0)
        # Actually, let's just use trimesh subdivide + normalize
        import trimesh
        tm = trimesh.Trimesh(vertices=verts, faces=[list(f) for f in faces])
        tm = tm.subdivide()
        # Project to unit sphere
        pts = []
        for v in tm.vertices:
            r = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
            if r > 1e-9:
                pts.append((v[0]/r, v[1]/r, v[2]/r))
        verts = pts
        faces = [tuple(int(i) for i in face) for face in tm.faces]
    return verts, faces
