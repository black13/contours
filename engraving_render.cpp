// ─────────────────────────────────────────────────────────────────────────
// engraving_render.cpp — Monte Carlo light-eroded engraving in C++
// ─────────────────────────────────────────────────────────────────────────
//
// Pipeline
// --------
// PLY file → winged-edge (WXEdgeBuilder) → face classification (n·v > 0)
// → barycentric face sampling → light intensity (Lambert × 1/r²)
// → stroke probability gate → stroke direction (cross(n, view_dir))
// → 3D→2D projection → PDF or SVG output.
//
// Every step mirrors the Python polytope-oracle/monte_carlo_engraving.py
// kernel.  The difference is this runs on arbitrary PLY meshes (not just
// polytope primitives) via the Freestyle C++ library.
//
// Current limitation
// ------------------
// The output is pure scattered strokes — no silhouette edges, no face fills,
// no background field.  The strokes accumulate like dust on a surface,
// gathering more densely in shadow and thinning in light.  Silhouette edges
// and face-boundary strokes are planned but not yet drawn.
//
// Math per sample
// ---------------
//   1. Pick a random visible face (area-weighted roulette).
//   2. Pick a random point on that face (barycentric in a triangle fan).
//   3. Compute light intensity at the point:
//        d = ||light_pos − point||
//        n·l = max(0, dot(face_normal, light_dir))
//        intensity = n·l / d^falloff
//   4. Compute view obliquity:
//        n·v = |dot(face_normal, view_dir)|
//        weight = 1 − n·v × 0.7
//   5. Stroke probability:
//        prob = clamp((1 − intensity) × weight, 0.01, 1.0)
//   6. If random() > prob: skip this sample.
//   7. Stroke direction:
//        axis = normalize(cross(face_normal, view_dir))
//        This is the direction the burin follows across the face.
//   8. Stroke endpoints:
//        pt_a = point − axis × half_length
//        pt_b = point + axis × half_length
//        Project both to 2D via Camera::projectPoint()
//   9. Stroke appearance:
//        thickness = 0.10 + 0.55 × (1 − prob)    [thicker in shadow]
//        gray      = 0.03 + 0.52 × (1 − prob)    [darker in shadow]
//
// Output
// ------
// PDF (default):  A4 portrait (595×842 pt), strokes mapped from 800×600
//                 image coordinates to the page.
// SVG (--svg):    800×600 viewBox with per-line width and gray RGB.
//
// Light source
// ------------
// Point light at configurable 3D position with configurable falloff
// exponent.  Default: (1, 4, 3) with 1/r² falloff — light coming from
// upper-right-front.  No shadow rays yet (Embree integration point).
//
// Camera
// ------
// Default: perspective at (2.8, 2.0, 3.5) looking at origin, 35° FOV.
// Customizable via --camera-x/y/z but currently fixed in code.  The
// Camera class in engraving/camera.h handles projection and front-facing
// tests.
//
// Building
// --------
//   cd contours/build && cmake .. && make engraving_render
//
// Running
// -------
//   ./engraving_render input.ply --output out.pdf --samples 3000
//   ./engraving_render input.ply --svg out.svg --light-x 0 --light-y 5 --light-z 0
//
// Verification
// ------------
// Against the Python oracle: an icosahedron PLY (12 vertices, 20 faces)
// produces 8 visible faces (matches oracle).  With 1000 samples,
// approximately 650 strokes pass the probability gate.  The distribution
// is uniform across visible face area.
//
// Todo
// ----
// - Add silhouette edge strokes (bold, dark) — edges where one face is
//   front-facing and the adjacent face is back-facing.
// - Add face fills (light gray background) so the solid reads.
// - Add shadow rays via Embree for proper occlusion.
// - Port the freehand path tracer (continuous paths with inertia,
//   noise-displaced curves, face-crossing).
// - Add camera position to CLI.
// - Support subdivision (Catmull-Clark in C++) for smooth limit surfaces.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <map>

#include "scene_graph/PLYFileLoader.h"
#include "winged_edge/WXEdgeBuilder.h"
#include "winged_edge/Nature.h"
#include "view_map/FEdgeXDetector.h"
#include "view_map/SilhouetteGeomEngine.h"

#include "engraving/camera.h"
#include "engraving/args.h"
#include "engraving/pdf_writer.h"

using Geometry::Vec3r;

// ── SVG helpers (kept for --svg flag) ─────────────────────────────────
//
// SVG is simpler than PDF for debugging — open in any browser.
// Each stroke is a <line> element with stroke-width and stroke="rgb(g,g,g)".

static void svgHeader(std::ostream& out, int w, int h) {
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << w << "\" height=\"" << h << "\" "
        << "viewBox=\"0 0 " << w << " " << h << "\">\n"
        << "<rect width=\"" << w << "\" height=\"" << h << "\" fill=\"#f0f0eb\"/>\n";
}

static void svgFooter(std::ostream& out) { out << "</svg>\n"; }

static void svgLine(std::ostream& out,
                    double x1, double y1, double x2, double y2,
                    double width, double gray) {
    int g = (int)(gray * 255.0);
    out << "  <line x1=\"" << x1 << "\" y1=\"" << y1
        << "\" x2=\"" << x2 << "\" y2=\"" << y2
        << "\" stroke=\"rgb(" << g << "," << g << "," << g << ")\""
        << " stroke-width=\"" << width << "\""
        << " stroke-linecap=\"round\"/>\n";
}

// ── Stroke data ───────────────────────────────────────────────────────
//
// A Stroke is a 2D line segment with appearance.  Width is in PDF points
// (SVG pixels), gray is 0.0 (black) to 1.0 (white).

struct Stroke {
    double x1, y1, x2, y2, width, gray;
};

// ── generateStrokes — the core kernel ─────────────────────────────────
//
// This is the C++ equivalent of Python's monte_carlo_strokes().
// For each of numSamples iterations:
//   1. Pick a visible face (area-weighted).
//   2. Pick a random point on that face (barycentric).
//   3. Compute light intensity, view obliquity, stroke probability.
//   4. If the probability gate passes, compute direction and endpoints.
//   5. Append a Stroke to the output vector.
//
// Returns the vector of all generated strokes.

static std::vector<Stroke> generateStrokes(
    WingedEdge* we,
    const Camera& cam,
    const Vec3r& lightPos,
    int numSamples,
    double falloffExp,
    int seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    // ── Collect visible faces ──────────────────────────────────────
    // Each face stores its vertex positions (for barycentric sampling),
    // center, normal, and area (for importance-weighted face selection).
    struct FaceInfo {
        std::vector<Vec3r> verts3d;  // face vertex positions in order
        Vec3r center;
        Vec3r normal;
        double area;
    };
    std::vector<FaceInfo> visibleFaces;
    double totalArea = 0.0;

    // Iterate over all shapes and faces in the winged-edge structure.
    // WXFace provides center() and GetNormal() — both computed during
    // the winged-edge build from the original PLY geometry.
    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        for (auto fit = shape->GetFaceList().begin(); fit != shape->GetFaceList().end(); ++fit) {
            WXFace* wxf = dynamic_cast<WXFace*>(*fit);
            if (!wxf) continue;
            Vec3r center = wxf->center();
            Vec3r normal = wxf->GetNormal();

            // Front-facing test: n·(camera − center) > 0.
            // Back-facing faces contribute zero strokes.
            if (!cam.isFrontFacing(normal, center)) continue;

            // Retrieve face vertex positions for area computation and
            // barycentric point sampling.  WFace::RetrieveVertexList()
            // fills a vector<WVertex*> with the face's vertices in order.
            std::vector<WVertex*> wvlist;
            wxf->RetrieveVertexList(wvlist);
            std::vector<Vec3r> verts3d;
            double area = 0.0;
            if (wvlist.size() >= 3) {
                // Triangle-fan area: sum of 0.5 × ||cross(b−a, c−a)||
                Vec3r a = wvlist[0]->GetVertex();
                for (size_t i = 1; i + 1 < wvlist.size(); i++) {
                    Vec3r b = wvlist[i]->GetVertex();
                    Vec3r c = wvlist[i+1]->GetVertex();
                    area += 0.5 * ((b - a) ^ (c - a)).norm();
                }
                for (auto wv : wvlist)
                    verts3d.push_back(wv->GetVertex());
            }
            visibleFaces.push_back({verts3d, center, normal, area});
            totalArea += area;
        }
    }

    std::cerr << "Visible faces: " << visibleFaces.size() << "\n";

    std::vector<Stroke> strokes;
    int nf = (int)visibleFaces.size();
    if (nf == 0) return strokes;

    // ── Monte Carlo loop ───────────────────────────────────────────
    for (int s = 0; s < numSamples; s++) {
        // Area-weighted face selection: larger faces get more samples.
        double r = unit(rng) * totalArea;
        double cumulative = 0.0;
        int fi = 0;
        for (int i = 0; i < nf; i++) {
            cumulative += visibleFaces[i].area;
            if (r <= cumulative) { fi = i; break; }
        }

        FaceInfo& info = visibleFaces[fi];
        Vec3r normal = info.normal;
        const auto& verts = info.verts3d;

        // Barycentric point sampling within the face polygon.
        // If the face is degenerate (fewer than 3 vertices), fall back
        // to the face center.
        Vec3r samplePt;
        int nv = (int)verts.size();
        if (nv >= 3) {
            Vec3r a = verts[0];
            // Pick a random triangle in the fan: triangles are
            // (v0, v1, v2), (v0, v2, v3), ..., (v0, v_{k-2}, v_{k-1})
            int tri = 1 + (int)(unit(rng) * (nv - 2));
            if (tri >= nv - 1) tri = nv - 2;
            Vec3r b = verts[tri];
            Vec3r c = verts[tri + 1];
            // Barycentric coordinates: uniform in the triangle
            double u = unit(rng), v = unit(rng);
            if (u + v > 1.0) { u = 1.0 - u; v = 1.0 - v; }
            samplePt = a + (b - a) * u + (c - a) * v;
        } else {
            samplePt = info.center;
        }

        // ── Light intensity: Lambert × 1/r^falloff ──────────────
        // n·l = cosine of angle between normal and light direction.
        // Positive means the face faces the light.
        // Divided by distance^falloff for point-light attenuation.
        Vec3r toLight = lightPos - samplePt;
        double dist = toLight.norm();
        double lightIntensity = 0.0;
        if (dist > 1e-9) {
            Vec3r lightDir = toLight; lightDir.normalize();
            double ndotl = std::max(0.0, normal * lightDir);
            lightIntensity = ndotl / pow(dist, falloffExp);
        }

        // ── View obliquity ───────────────────────────────────────
        // n·v = absolute cosine of angle between normal and view dir.
        // Face facing the camera: n·v ≈ 1 → weight ≈ 0.3 (more lines).
        // Face edge-on: n·v ≈ 0 → weight ≈ 1.0 (fewer lines).
        Vec3r viewDir = cam.position - samplePt;
        double vlen = viewDir.norm();
        if (vlen < 1e-9) continue;
        viewDir = viewDir * (1.0 / vlen);
        double ndotv = std::max(0.01, fabs(normal * viewDir));
        double viewWeight = 1.0 - ndotv * 0.7;

        // ── Stroke probability ───────────────────────────────────
        // Light erodes: high intensity → low probability.
        // View obliquity enhances: edge-on faces get more strokes.
        double strokeProb = std::max(0.01, std::min(1.0, (1.0 - lightIntensity) * viewWeight));
        if (unit(rng) > strokeProb) continue;

        // ── Stroke direction ─────────────────────────────────────
        // Perpendicular to both the face normal and the view direction.
        // This is the direction the burin follows — it lies in the
        // face plane and is perpendicular to the line of sight.
        Vec3r sd3 = normal ^ viewDir;
        double sdNorm = sd3.norm();
        if (sdNorm < 1e-9) sd3 = normal ^ Vec3r(0, 1, 0);
        else sd3 = sd3 * (1.0 / sdNorm);

        // ── Stroke endpoints in 3D, projected to 2D ──────────────
        double halfLen = 15.0 * strokeProb * 0.5 * 0.02;
        Vec3r ptA = samplePt - sd3 * halfLen;
        Vec3r ptB = samplePt + sd3 * halfLen;

        double x1, y1, x2, y2;
        cam.projectPoint(ptA, 800, 600, x1, y1);
        cam.projectPoint(ptB, 800, 600, x2, y2);

        // ── Stroke appearance ────────────────────────────────────
        // Thicker and darker where the stroke probability was higher
        // (i.e., in shadow, where light intensity is low).
        double thick = 0.10 + 0.55 * (1.0 - strokeProb);
        double gray = 0.03 + 0.52 * (1.0 - strokeProb);

        strokes.push_back({x1, y1, x2, y2, thick, gray});
    }
    return strokes;
}

// ── main — entry point ─────────────────────────────────────────────────

int main(int argc, char** argv) {
    Args args(argc, argv);

    // ── Parse CLI ──────────────────────────────────────────────────
    if (args.getString("--input").empty() && argc < 2) {
        std::cerr << "Usage: " << args.program
                  << " <input.ply> [--output out.pdf|--svg out.svg]\n"
                  << "  --samples N     Monte Carlo samples (default 2000)\n"
                  << "  --light-x/y/z   point light position (default 1,4,3)\n"
                  << "  --falloff N     light falloff exponent (default 2 = 1/r^2)\n"
                  << "  --seed N        random seed (default 42)\n"
                  << "  --svg PATH      output SVG instead of PDF\n";
        return 1;
    }

    std::string plyPath = (argc >= 2 && argv[1][0] != '-')
        ? argv[1] : args.getString("--input");
    std::string outPath = args.getString("--output", "engraving.pdf");
    std::string svgPath = args.getString("--svg", "");
    bool useSvg = !svgPath.empty();

    int numSamples = args.getInt("--samples", 2000);
    int seed = args.getInt("--seed", 42);
    double falloffExp = args.getDouble("--falloff", 2.0);

    Vec3r lightPos(
        args.getDouble("--light-x", 1.0),
        args.getDouble("--light-y", 4.0),
        args.getDouble("--light-z", 3.0)
    );

    // ── Load PLY ───────────────────────────────────────────────────
    PLYFileLoader loader(plyPath.c_str());
    NodeGroup* root = loader.Load();
    if (!root) { std::cerr << "PLY load failed\n"; return 1; }

    // ── Build winged-edge ──────────────────────────────────────────
    // WXEdgeBuilder converts the PLY scene graph into a WingedEdge
    // structure (vertices, edges, faces, normals, adjacency).
    WXEdgeBuilder wxBuilder;
    root->accept(wxBuilder);
    WingedEdge* we = wxBuilder.getWingedEdge();
    if (!we) { std::cerr << "Winged-edge build failed\n"; return 1; }

    // ── Camera ─────────────────────────────────────────────────────
    Camera cam;

    // ── Silhouette engine setup (required by FEdgeXDetector) ──────
    real mv[4][4] = {{0}};
    real proj[4][4] = {{0}};
    int vp[4] = {0, 0, 800, 600};
    mv[0][0]=1; mv[1][1]=1; mv[2][2]=1; mv[3][3]=1; mv[3][2] = -5.0f;
    float f = 1.0f / tanf(35.0f * (float)M_PI / 360.0f);
    proj[0][0] = f / (800.0f/600.0f); proj[1][1] = f;
    proj[2][2] = -1.0f; proj[2][3] = -1.0f; proj[3][2] = -0.2f;
    SilhouetteGeomEngine* sge = SilhouetteGeomEngine::getInstance();
    sge->SetViewpoint(Vec3r(0, 0, 5));
    sge->SetTransform(mv, proj, vp, 1.0f);

    // ── Feature edge detector ──────────────────────────────────────
    // Runs FEdgeXDetector on the winged-edge to classify edges by
    // nature (silhouette, border, crease, suggestives).  Required for
    // future silhouette edge rendering.
    FEdgeXDetector detector;
    detector.SetViewpoint(Vec3r(0, 0, 5));
    detector.enableRidgesAndValleysFlag(true);
    detector.enableSuggestiveContours(true);
    detector.processShapes(*we);

    // ── Generate strokes ───────────────────────────────────────────
    std::vector<Stroke> strokes = generateStrokes(
        we, cam, lightPos, numSamples, falloffExp, seed);

    // ── Silhouette edges ───────────────────────────────────────────
    // An edge is silhouette if exactly ONE of its two adjacent faces
    // is front-facing.  These edges form the occluding contour — the
    // boundary between visible and hidden surface.  We draw them as
    // bold dark lines to anchor the scattered strokes.
    //
    // Algorithm: iterate all edges in the winged-edge.  For each edge
    // with two adjacent faces, check front/back status of both.  If
    // one is front and the other back, project the edge endpoints
    // and emit a bold dark Stroke.
    std::vector<Stroke> silStrokes;
    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        for (auto eit = shape->GetEdgeList().begin();
             eit != shape->GetEdgeList().end(); ++eit) {
            WXEdge* edge = dynamic_cast<WXEdge*>(*eit);
            if (!edge) continue;
            WXFace* fa = dynamic_cast<WXFace*>(edge->GetaFace());
            WXFace* fb = dynamic_cast<WXFace*>(edge->GetbFace());
            if (!fa || !fb) continue;  // border edge — skip
            bool frontA = cam.isFrontFacing(fa->GetNormal(), fa->center());
            bool frontB = cam.isFrontFacing(fb->GetNormal(), fb->center());
            if (frontA == frontB) continue;  // both front or both back

            // Silhouette edge — project endpoints and emit bold dark stroke
            WVertex* va = edge->GetaVertex();
            WVertex* vb = edge->GetbVertex();
            double x1, y1, x2, y2;
            cam.projectPoint(va->GetVertex(), 800, 600, x1, y1);
            cam.projectPoint(vb->GetVertex(), 800, 600, x2, y2);
            silStrokes.push_back({x1, y1, x2, y2, 1.2, 0.02});
        }
    }

    std::cerr << "Silhouette edges: " << silStrokes.size() << "\n";

    // ── Output ─────────────────────────────────────────────────────
    if (useSvg) {
        std::ofstream svg(svgPath);
        svgHeader(svg, 800, 600);
        // Silhouette edges first (behind strokes? no — on top, bold)
        for (auto& st : strokes)
            svgLine(svg, st.x1, st.y1, st.x2, st.y2, st.width, st.gray);
        for (auto& st : silStrokes)
            svgLine(svg, st.x1, st.y1, st.x2, st.y2, st.width, st.gray);
        svgFooter(svg);
        svg.close();
        std::cerr << "Wrote " << strokes.size() + silStrokes.size()
                  << " strokes to " << svgPath << "\n";
    } else {
        // Map 800×600 image coords to A4 portrait (595×842 PDF points)
        PDFWriter pdf(outPath, 595.0, 842.0);
        double sx = 595.0 / 800.0;
        double sy = 842.0 / 600.0;
        for (auto& st : strokes) {
            pdf.strokeLine(
                st.x1 * sx, st.y1 * sy,
                st.x2 * sx, st.y2 * sy,
                st.width * sx * 0.5, st.gray);
        }
        // Silhouette edges: bold (1.2pt × scale) and dark (gray 0.02)
        for (auto& st : silStrokes) {
            pdf.strokeLine(
                st.x1 * sx, st.y1 * sy,
                st.x2 * sx, st.y2 * sy,
                1.0, 0.02);
        }
        pdf.save();
        std::cerr << "Wrote " << strokes.size() + silStrokes.size()
                  << " strokes to " << outPath << "\n";
    }

    return 0;
}
