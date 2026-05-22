// engraving_render.cpp — Monte Carlo light-eroded engraving in C++
// =================================================================
//
// Loads PLY → builds winged-edge → classifies faces → Monte Carlo
// stroke generation → output PDF or SVG.
//
// Usage:
//   engraving_render input.ply --output out.pdf --samples 3000
//   engraving_render input.ply --svg out.svg  --light-x 1 --light-y 4 --light-z 3
//
// Depends on the contours Freestyle library and our engraving/
// helpers (camera.h, args.h, pdf_writer.h).

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

// ── Monte Carlo stroke generation ─────────────────────────────────────

struct Stroke {
    double x1, y1, x2, y2, width, gray;
};

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

    // Collect visible faces
    struct FaceInfo {
        Vec3r center;
        Vec3r normal;
        double ndotl;
    };
    std::vector<FaceInfo> visibleFaces;

    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        for (auto fit = shape->GetFaceList().begin(); fit != shape->GetFaceList().end(); ++fit) {
            WXFace* wxf = dynamic_cast<WXFace*>(*fit);
            if (!wxf) continue;
            Vec3r center = wxf->center();
            Vec3r normal = wxf->GetNormal();
            if (!cam.isFrontFacing(normal, center)) continue;

            Vec3r toLight = lightPos - center;
            double dist = toLight.norm();
            double ndotl = 0.0;
            if (dist > 1e-9) {
                Vec3r lightDir = toLight; lightDir.normalize();
                ndotl = std::max(0.0, normal * lightDir);
            }
            visibleFaces.push_back({center, normal, ndotl});
        }
    }

    std::cerr << "Visible faces: " << visibleFaces.size() << "\n";

    std::vector<Stroke> strokes;
    int nf = (int)visibleFaces.size();
    if (nf == 0) return strokes;

    for (int s = 0; s < numSamples; s++) {
        int fi = (int)(unit(rng) * nf);
        if (fi >= nf) fi = nf - 1;

        FaceInfo& info = visibleFaces[fi];
        Vec3r center = info.center;
        Vec3r normal = info.normal;

        // Light intensity: Lambert × 1/r^falloff
        Vec3r toLight = lightPos - center;
        double dist = toLight.norm();
        double lightIntensity = 0.0;
        if (dist > 1e-9) {
            Vec3r lightDir = toLight; lightDir.normalize();
            double ndotl = std::max(0.0, normal * lightDir);
            lightIntensity = ndotl / pow(dist, falloffExp);
        }

        // View obliquity
        Vec3r viewDir = cam.position - center;
        double vlen = viewDir.norm();
        if (vlen < 1e-9) continue;
        viewDir = viewDir * (1.0 / vlen);
        double ndotv = std::max(0.01, fabs(normal * viewDir));
        double viewWeight = 1.0 - ndotv * 0.7;

        // Stroke probability
        double strokeProb = std::max(0.01, std::min(1.0, (1.0 - lightIntensity) * viewWeight));
        if (unit(rng) > strokeProb) continue;

        // Stroke direction: cross(normal, viewDir)
        Vec3r sd3 = normal ^ viewDir;
        double sdNorm = sd3.norm();
        if (sdNorm < 1e-9) sd3 = normal ^ Vec3r(0, 1, 0);
        else sd3 = sd3 * (1.0 / sdNorm);

        double halfLen = 15.0 * strokeProb * 0.5 * 0.02;
        Vec3r ptA = center - sd3 * halfLen;
        Vec3r ptB = center + sd3 * halfLen;

        double x1, y1, x2, y2;
        cam.projectPoint(ptA, 800, 600, x1, y1);
        cam.projectPoint(ptB, 800, 600, x2, y2);

        double thick = 0.10 + 0.55 * (1.0 - strokeProb);
        double gray = 0.03 + 0.52 * (1.0 - strokeProb);

        strokes.push_back({x1, y1, x2, y2, thick, gray});
    }
    return strokes;
}

// ── main ──────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    Args args(argc, argv);

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

    // ── Load PLY ──
    PLYFileLoader loader(plyPath.c_str());
    NodeGroup* root = loader.Load();
    if (!root) { std::cerr << "PLY load failed\n"; return 1; }

    // ── Build winged-edge ──
    WXEdgeBuilder wxBuilder;
    root->accept(wxBuilder);
    WingedEdge* we = wxBuilder.getWingedEdge();
    if (!we) { std::cerr << "Winged-edge build failed\n"; return 1; }

    // ── Camera ──
    Camera cam;

    // ── Set up silhouette engine (needed by detector) ──
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

    // ── Run detector ──
    FEdgeXDetector detector;
    detector.SetViewpoint(Vec3r(0, 0, 5));
    detector.enableRidgesAndValleysFlag(true);
    detector.enableSuggestiveContours(true);
    detector.processShapes(*we);

    // ── Generate strokes ──
    std::vector<Stroke> strokes = generateStrokes(
        we, cam, lightPos, numSamples, falloffExp, seed);

    // ── Output ──
    if (useSvg) {
        std::ofstream svg(svgPath);
        svgHeader(svg, 800, 600);
        for (auto& st : strokes)
            svgLine(svg, st.x1, st.y1, st.x2, st.y2, st.width, st.gray);
        svgFooter(svg);
        svg.close();
        std::cerr << "Wrote " << strokes.size() << " strokes to " << svgPath << "\n";
    } else {
        PDFWriter pdf(outPath, 595.0, 842.0);
        double sx = 595.0 / 800.0;
        double sy = 842.0 / 600.0;
        for (auto& st : strokes) {
            pdf.strokeLine(
                st.x1 * sx, st.y1 * sy,
                st.x2 * sx, st.y2 * sy,
                st.width * sx * 0.5, st.gray);
        }
        pdf.save();
        std::cerr << "Wrote " << strokes.size() << " strokes to " << outPath << "\n";
    }

    return 0;
}
