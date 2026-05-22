// engraving_render.cpp — Monte Carlo light-eroded engraving in C++
//
// Loads a PLY, builds winged-edge, classifies faces by front/back,
// then generates strokes using Monte Carlo sampling on visible faces.
// Light erodes darkness via Lambert × 1/r² falloff.
// Output is SVG with varying stroke-width and gray per line.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include "scene_graph/PLYFileLoader.h"
#include "winged_edge/WXEdgeBuilder.h"
#include "winged_edge/Nature.h"
#include "view_map/FEdgeXDetector.h"
#include "view_map/SilhouetteGeomEngine.h"

using Geometry::Vec3r;

// ── SVG helpers ──────────────────────────────────────────────────────

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

// ── Perspective projection ───────────────────────────────────────────

static void projectPoint(const Vec3r& pt, const Vec3r& camPos,
                         const Vec3r& forward, const Vec3r& right, const Vec3r& up,
                         int imgW, int imgH, float fovY,
                         double& outX, double& outY) {
    Vec3r rel = pt - camPos;
    double x = rel * right;
    double y = rel * up;
    double z = rel * forward;
    if (z < 1e-6) z = 1e-6;
    double aspect = (double)imgW / (double)imgH;
    double tanHalfFov = tan(fovY * M_PI / 360.0);
    double ndcX = x / (z * tanHalfFov * aspect);
    double ndcY = y / (z * tanHalfFov);
    outX = (ndcX + 1.0) * 0.5 * imgW;
    outY = (1.0 - (ndcY + 1.0) * 0.5) * imgH;
}

// ── Main ──────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.ply> [--svg out.svg] [--samples N]"
                  << " [--light-x X] [--light-y Y] [--light-z Z] [--seed S]\n";
        return 1;
    }

    std::string plyPath = argv[1];
    std::string svgPath = "engraving.svg";
    int numSamples = 2000;
    Vec3r lightPos(1.0, 4.0, 3.0);
    int seed = 42;
    double falloffExp = 2.0;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--svg" && i + 1 < argc) svgPath = argv[++i];
        else if (arg == "--samples" && i + 1 < argc) numSamples = atoi(argv[++i]);
        else if (arg == "--light-x" && i + 1 < argc) lightPos[0] = atof(argv[++i]);
        else if (arg == "--light-y" && i + 1 < argc) lightPos[1] = atof(argv[++i]);
        else if (arg == "--light-z" && i + 1 < argc) lightPos[2] = atof(argv[++i]);
        else if (arg == "--seed" && i + 1 < argc) seed = atoi(argv[++i]);
        else if (arg == "--falloff" && i + 1 < argc) falloffExp = atof(argv[++i]);
    }

    // 1. Load PLY
    PLYFileLoader loader(plyPath.c_str());
    NodeGroup* root = loader.Load();
    if (!root) { std::cerr << "PLY load failed\n"; return 1; }

    // 2. Build winged-edge
    WXEdgeBuilder wxBuilder;
    root->accept(wxBuilder);
    WingedEdge* we = wxBuilder.getWingedEdge();
    if (!we) { std::cerr << "Winged-edge build failed\n"; return 1; }

    // 3. Set up camera + silhouette engine
    Vec3r camPos(2.8, 2.0, 3.5);
    Vec3r camTarget(0, 0, 0);
    Vec3r forward = camTarget - camPos; forward.normalize();
    Vec3r worldUp(0, 1, 0);
    Vec3r rgt = forward ^ worldUp; rgt.normalize();
    Vec3r up = rgt ^ forward; up.normalize();
    int imgW = 800, imgH = 600;
    float fovY = 35.0f;

    // Set up SilhouetteGeomEngine (needed by FEdgeXDetector, even
    // though we compute front-facing ourselves)
    real mv[4][4] = {{0}};
    real proj[4][4] = {{0}};
    int vp[4] = {0, 0, imgW, imgH};
    mv[0][0]=1; mv[1][1]=1; mv[2][2]=1; mv[3][3]=1;
    mv[3][2] = -5.0f;
    float aspect = (float)imgW / (float)imgH;
    float f = 1.0f / tanf(fovY * (float)M_PI / 360.0f);
    proj[0][0] = f / aspect; proj[1][1] = f;
    proj[2][2] = -1.0f; proj[2][3] = -1.0f;
    proj[3][2] = -2.0f * 0.1f * 100.0f / (100.0f - 0.1f);
    SilhouetteGeomEngine* sge = SilhouetteGeomEngine::getInstance();
    sge->SetViewpoint(Vec3r(0, 0, 5));
    sge->SetTransform(mv, proj, vp, 1.0f);

    // 4. Run detector (needed for edge classification on silhouette edges)
    FEdgeXDetector detector;
    detector.SetViewpoint(Vec3r(0, 0, 5));
    detector.enableRidgesAndValleysFlag(true);
    detector.enableSuggestiveContours(true);
    detector.processShapes(*we);

    // 5. Collect visible faces with their centers, normals, and areas
    struct FaceInfo {
        int id;
        Vec3r center;
        Vec3r normal;
        double area;
        double ndotl;  // light dot product (for sorting/stats)
        WXFace* ptr;
    };
    std::vector<FaceInfo> visibleFaces;
    double totalArea = 0.0;

    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        for (auto fit = shape->GetFaceList().begin(); fit != shape->GetFaceList().end(); ++fit) {
            WXFace* wxf = dynamic_cast<WXFace*>(*fit);
            if (!wxf) continue;
            Vec3r center = wxf->center();
            Vec3r normal = wxf->GetNormal();
            Vec3r viewDir = camPos - center;
            if (normal * viewDir <= 0) continue;  // back-facing

            // Light on this face center
            Vec3r toLight = lightPos - center;
            double dist = toLight.norm();
            double ndotl = 0.0;
            if (dist > 1e-9) {
                Vec3r lightDir = toLight; lightDir.normalize();
                ndotl = std::max(0.0, normal * lightDir);
            }

            // Approximate area from the winged-edge face bounding edges
            // For now, use 1.0 as placeholder — uniform sampling
            double area = 1.0;

            FaceInfo fi = {wxf->GetId(), center, normal, area, ndotl, wxf};
            visibleFaces.push_back(fi);
            totalArea += area;
        }
    }

    if (visibleFaces.empty()) {
        std::cerr << "No visible faces\n";
        return 0;
    }

    std::cerr << "Visible faces: " << visibleFaces.size()
              << "  Light at (" << lightPos[0] << "," << lightPos[1] << "," << lightPos[2] << ")\n";

    // 6. Monte Carlo stroke generation
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    struct Stroke {
        double x1, y1, x2, y2, width, gray;
    };
    std::vector<Stroke> strokes;

    for (int s = 0; s < numSamples; s++) {
        // Pick a random visible face — weighted by area for importance sampling
        double r = unit(rng) * totalArea;
        double cumulative = 0.0;
        int chosen = 0;
        for (int i = 0; i < (int)visibleFaces.size(); i++) {
            cumulative += visibleFaces[i].area;
            if (r <= cumulative) { chosen = i; break; }
        }

        FaceInfo& fi = visibleFaces[chosen];
        Vec3r center = fi.center;
        Vec3r normal = fi.normal;

        // Light intensity at face center: Lambert × 1/r^falloff
        Vec3r toLight = lightPos - center;
        double dist = toLight.norm();
        double lightIntensity = 0.0;
        if (dist > 1e-9) {
            Vec3r lightDir = toLight; lightDir.normalize();
            double ndotl = std::max(0.0, normal * lightDir);
            lightIntensity = ndotl / pow(dist, falloffExp);
        }

        // View obliquity
        Vec3r viewDir = camPos - center;
        double vlen = viewDir.norm();
        if (vlen < 1e-9) continue;
        viewDir = viewDir * (1.0 / vlen);
        double ndotv = std::max(0.01, fabs(normal * viewDir));
        double viewWeight = 1.0 - ndotv * 0.7;

        // Stroke probability: light erodes, obliquity enhances
        double strokeProb = std::max(0.01, std::min(1.0, (1.0 - lightIntensity) * viewWeight));
        if (unit(rng) > strokeProb) continue;

        // Stroke direction: cross(normal, viewDir) — follows face grain
        Vec3r sd3 = normal ^ viewDir;
        double sdNorm = sd3.norm();
        if (sdNorm < 1e-9) sd3 = normal ^ Vec3r(0, 1, 0);
        else sd3 = sd3 * (1.0 / sdNorm);

        // Stroke endpoints
        double halfLen = 15.0 * strokeProb * 0.5 * 0.02;
        Vec3r ptA = center - sd3 * halfLen;
        Vec3r ptB = center + sd3 * halfLen;

        double x1, y1, x2, y2;
        projectPoint(ptA, camPos, forward, rgt, up, imgW, imgH, fovY, x1, y1);
        projectPoint(ptB, camPos, forward, rgt, up, imgW, imgH, fovY, x2, y2);

        double thick = 0.10 + 0.55 * (1.0 - strokeProb);
        double gray = 0.03 + 0.52 * (1.0 - strokeProb);

        strokes.push_back({x1, y1, x2, y2, thick, gray});
    }

    // 7. Write SVG
    std::ofstream svg(svgPath);
    svgHeader(svg, imgW, imgH);
    for (auto& st : strokes)
        svgLine(svg, st.x1, st.y1, st.x2, st.y2, st.width, st.gray);
    svgFooter(svg);
    svg.close();

    std::cerr << "Wrote " << strokes.size() << " strokes to " << svgPath << "\n";
    return 0;
}
