// freestyle_render.cpp — PLY → winged-edge → FEdgeXDetector → dump feature edges
// With camera setup for curvature-based features (ridges, valleys, suggestive)

#include <iostream>
#include <string>
#include <map>
#include <cmath>
#include "scene_graph/PLYFileLoader.h"
#include "winged_edge/WXEdgeBuilder.h"
#include "winged_edge/Nature.h"
#include "view_map/FEdgeXDetector.h"
#include "view_map/SilhouetteGeomEngine.h"

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.ply>\n";
        return 1;
    }

    // 1. Load PLY
    PLYFileLoader loader(argv[1]);
    NodeGroup* root = loader.Load();
    if (!root) { std::cerr << "PLY load failed\n"; return 1; }
    std::cerr << "Loaded PLY: " << root->numberOfChildren() << " nodes\n";

    // 2. Build winged-edge
    WXEdgeBuilder wxBuilder;
    root->accept(wxBuilder);
    WingedEdge* we = wxBuilder.getWingedEdge();
    if (!we) { std::cerr << "Winged-edge build failed\n"; return 1; }

    // 3. Set up camera geometry (needed for curvature features)
    real mv[4][4] = {{0}};
    real proj[4][4] = {{0}};
    int vp[4] = {0, 0, 640, 480};
    float fov = 45.0f, near = 0.1f, far = 100.0f, aspect = 640.0f/480.0f;
    float f = 1.0f / tanf(fov * (float)M_PI / 360.0f);

    // Simple view: camera at (0,0,5) looking at origin
    mv[0][0]=1; mv[1][1]=1; mv[2][2]=1; mv[3][3]=1;
    mv[3][2] = -5.0f;  // translate by -5 in Z (column-major)

    // Perspective projection
    proj[0][0] = f / aspect;
    proj[1][1] = f;
    proj[2][2] = -(far+near)/(far-near);
    proj[2][3] = -1;
    proj[3][2] = -2*far*near/(far-near);

    SilhouetteGeomEngine* sge = SilhouetteGeomEngine::getInstance();
    sge->SetViewpoint(Vec3r(0, 0, 5));
    sge->SetTransform(mv, proj, vp, 1.0f);
    sge->SetFrustum(near, far);

    // 4. Feature-edge detector
    std::cerr << "\nFEdgeXDetector with camera: ridges=ON valleys=ON suggestive=ON\n";
    FEdgeXDetector detector;
    detector.enableRidgesAndValleysFlag(true);
    detector.enableSuggestiveContours(true);
    detector.setSphereRadius(1.0f);

    detector.processShapes(*we);

    // DEBUG: dump curvature info for first few vertices
    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        auto& verts = shape->GetVertexList();
        std::cerr << "Curvature sample (first 5 of " << verts.size() << " vertices):\n";
        int vcount = 0;
        for (auto vit = verts.begin(); vit != verts.end() && vcount < 5; ++vit, ++vcount) {
            WXVertex* wxv = dynamic_cast<WXVertex*>(*vit);
            if (!wxv || !wxv->curvatures()) continue;
            auto* c = wxv->curvatures();
            std::cerr << "  v" << vcount << ": K1=" << c->K1 << " K2=" << c->K2
                      << " Kr=" << c->Kr << " dKr=" << c->dKr
                      << " e1=(" << c->e1[0] << "," << c->e1[1] << "," << c->e1[2] << ")\n";
        }
        break; // just one shape
    }
    std::cerr << "\n";

    // 4. Walk every edge, classify by Nature bitmask
    std::map<std::string, int> counts;
    int totalEdges = 0;

    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        for (auto eit = shape->GetEdgeList().begin();
             eit != shape->GetEdgeList().end(); ++eit) {
            WXEdge* edge = dynamic_cast<WXEdge*>(*eit);
            if (!edge) continue;
            totalEdges++;

            unsigned n = edge->nature();
            std::string label;
            if (n & Nature::SILHOUETTE)          label += "SILHOUETTE ";
            if (n & Nature::BORDER)              label += "BORDER ";
            if (n & Nature::CREASE)              label += "CREASE ";
            if (n & Nature::RIDGE)               label += "RIDGE ";
            if (n & Nature::VALLEY)              label += "VALLEY ";
            if (n & Nature::SUGGESTIVE_CONTOUR)  label += "SUGGESTIVE ";
            if (label.empty()) label = "(none)";

            counts[label]++;

            if (counts[label] <= 4) {
                WVertex* a = edge->GetaVertex();
                WVertex* b = edge->GetbVertex();
                Vec3r pa = a->GetVertex();
                Vec3r pb = b->GetVertex();
                std::cerr << "  [" << label << "]"
                          << " A=(" << pa[0] << "," << pa[1] << "," << pa[2]
                          << ") B=(" << pb[0] << "," << pb[1] << "," << pb[2] << ")\n";
            }
        }
    }

    std::cerr << "\n=== EDGE CLASSIFICATION ===\n";
    std::cerr << "Total edges: " << totalEdges << "\n";
    for (auto& kv : counts)
        std::cerr << "  " << kv.first << ": " << kv.second << "\n";

    return 0;
}
