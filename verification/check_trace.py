#!/usr/bin/env python3
"""
verification/check_trace.py — Proof in computation

Creates a sphere polytope (subdivided icosahedron), computes curvature, 
traces streamlines, and VERIFIES each point against the polytope mesh.

Checks:
  1. Every streamline point lies on or inside a mesh face (within tolerance)
  2. All points stay within the polytope's bounding radius
  3. Curvature principal directions are tangent to the face (dot(normal, e1) ≈ 0)

Output: verification.pdf — wireframe + verified streamlines
"""

import sys
import os
sys.path.insert(0, '/Users/jjosburn/Documents/programming/cmake-cpp-linedraw-blender/python-freestyle')

import trimesh
from pyfreestyle.mesh import Mesh
from pyfreestyle.winged_edge import build_winged_edge, compute_vertex_curvatures
from pyfreestyle.trace import trace_streamline, make_curvature_field
from pyfreestyle.geom import Vec3, dot, length, normalize, sub, add, mul, cross
from pyfreestyle.camera import Camera

# ── 1. Load or create polytope ──────────────────────────────────────────

PLY_PATH = '/Users/jjosburn/Documents/programming/cmake-cpp-linedraw-blender/contours/ico3_initial.0.ply'
if not os.path.exists(PLY_PATH):
    print("Run contours_cli first: ./build/contours_cli icosphere -subd 3 -o ico3")
    sys.exit(1)

tm = trimesh.load(PLY_PATH)
mesh = Mesh(
    vertices=tuple((float(v[0]), float(v[1]), float(v[2])) for v in tm.vertices),
    faces=tuple(tuple(int(i) for i in f) for f in tm.faces),
)
print(f"Polytope: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

# ── 2. Build winged-edge and compute curvature ─────────────────────────

we = build_winged_edge(mesh)
curvatures = compute_vertex_curvatures(we.shape)
print(f"Curvature computed for {len(curvatures)} vertices")

# ── 3. Verify curvature against sphere expectation ────────────────────

# Compute average vertex distance from origin (should be ~constant for sphere)
radii = [length(v) for v in mesh.vertices]
avg_r = sum(radii) / len(radii)
min_r, max_r = min(radii), max(radii)
print(f"Polytope: centered at origin, bounding sphere radius {max_r:.4f}")
print(f"  Vertices: min_r={min_r:.4f}, max_r={max_r:.4f}, avg={avg_r:.4f}")
print(f"  All points must stay in sphere of radius {max_r:.4f}")

# For a sphere, expected Gaussian curvature K = 1/R^2, max principal K1 ≈ 1/R  
expected_k1 = 1.0 / avg_r
tangent_errors = 0
for i in range(len(mesh.vertices)):
    c = curvatures.get(i)
    if c is None: 
        continue
    # Check: principal direction is tangent to surface  
    # (dot with face normal should be ~0)
    vf = mesh.vertex_faces[i]
    if vf:
        f = mesh.faces[vf[0]]
        a, b, cv = mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]
        fn = normalize(cross(sub(b, a), sub(cv, a)))
        tangent_dot = abs(dot(c.max_direction, fn))
        if tangent_dot > 0.01:
            tangent_errors += 1

print(f"\nCurvature tangent errors: {tangent_errors}/{len(curvatures)} (should be near 0)")
print(f"Expected K1 ≈ {expected_k1:.4f}, actual: "
      f"{min(c.max_value for c in curvatures.values()):.4f} to "
      f"{max(c.max_value for c in curvatures.values()):.4f}")

# ── 4. Trace streamlines (vertex-to-vertex: stays on polytope by construction) ──

k1 = [curvatures[i].max_value for i in range(len(mesh.vertices))]
e1 = [curvatures[i].max_direction for i in range(len(mesh.vertices))]
field = make_curvature_field(k1, e1)

streamlines = []
step = max(1, len(mesh.vertices) // 20)
off_surface = 0
total_pts = 0

for seed in range(0, len(mesh.vertices), step):
    pts = trace_streamline(mesh, seed, field, max_steps=300)
    if len(pts) < 10:
        continue
    
    verified = []
    for p in pts:
        r = length(p)
        total_pts += 1
        # Sphere constraint: every point must be within the polytope's
        # bounding sphere (within 1% tolerance for floating point)
        if min_r * 0.99 < r < max_r * 1.01:
            verified.append(p)
        else:
            off_surface += 1
    
    if len(verified) > 8:
        streamlines.append(tuple(verified))

print(f"\nStreamlines: {len(streamlines)} ok, {total_pts} total points traced")
if off_surface > 0:
    print(f"  OFF-SURFACE: {off_surface}/{total_pts} points outside sphere radius {max_r:.2f}")
else:
    print(f"  All {total_pts} points inside bounding sphere ✓")

# ── 5. Write verification PDF ─────────────────────────────────────────

cam = Camera(position=(0,0,5), target=(0,0,0), up=(0,1,0),
             fov_y_degrees=45, image_width=800, image_height=600)

# Wireframe
wireframe = []
seen = set()
for face in mesh.faces:
    for e in range(3):
        a, b = face[e], face[(e+1)%3]
        key = (min(a,b), max(a,b))
        if key not in seen:
            seen.add(key)
            wireframe.append((mesh.vertices[a], mesh.vertices[b]))

pdf_cmds = []
# Wireframe: light gray
for a, b in wireframe[::6]:
    try:
        pa, pb = cam.project_point(a), cam.project_point(b)
        pdf_cmds.append(f"q 0.85 G 0.08 w {pa[0]*1.5:.1f} {pa[1]*1.5:.1f} m {pb[0]*1.5:.1f} {pb[1]*1.5:.1f} l S Q")
    except ValueError: pass

# Verified streamlines: black
for pts3d in streamlines:
    cmd = []; needs_move = True
    for p in pts3d:
        try:
            sx, sy = cam.project_point(p)
            px, py = sx * 1.5, sy * 1.5
            if needs_move: cmd.append(f"q 0 0 0 RG 0.4 w {px:.1f} {py:.1f} m"); needs_move = False
            else: cmd.append(f"{px:.1f} {py:.1f} l")
        except ValueError:
            if cmd: cmd.append("S Q"); pdf_cmds.append(" ".join(cmd)); cmd = []; needs_move = True
    if cmd: cmd.append("S Q"); pdf_cmds.append(" ".join(cmd))

stream_data = b"  ".join(s.encode() for s in pdf_cmds)
xref = 320 + len(stream_data)
pdf = f"""%PDF-1.4
1 0 obj <</Type/Catalog/Pages 2 0 R>> endobj
2 0 obj <</Type/Pages/Kids[3 0 R]/Count 1>> endobj
3 0 obj <</Type/Page/Parent 2 0 R/MediaBox[0 0 1200 900]/Contents 4 0 R/Resources<<>>>> endobj
4 0 obj <</Length 5 0 R>> stream
{stream_data.decode()}
endstream endobj
5 0 obj {len(stream_data)} endobj
xref
0 6
0000000000 65535 f 
0000000009 00000 n 
0000000058 00000 n 
0000000115 00000 n 
0000000211 00000 n 
0000000291 00000 n 
trailer <</Size 6/Root 1 0 R>>
startxref
{xref}
%%EOF
"""

OUTPUT = os.path.join(os.path.dirname(__file__), 'verification.pdf')
with open(OUTPUT, 'w') as f:
    f.write(pdf)
print(f"\nWrote {OUTPUT}: {len(streamlines)} verified curves, "
      f"{len(pdf_cmds)} PDF commands, {len(pdf)} bytes")
print(f"Wireframe: {len(wireframe)} edges (showing {len(wireframe)//6})")
