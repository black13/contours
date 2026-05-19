// freestyle_render.cpp — PLY → winged-edge → FEdgeXDetector → oracle-comparable JSON
// Extended to emit front-facing faces, silhouette edges, and visibility data
// matching the Python polytope oracle output format.

#include <iostream>
#include <string>
#include <map>
#include <cmath>
#include <cstdio>
#include <sstream>
#include "scene_graph/PLYFileLoader.h"
#include "winged_edge/WXEdgeBuilder.h"
#include "winged_edge/Nature.h"
#include "view_map/FEdgeXDetector.h"
#include "view_map/SilhouetteGeomEngine.h"
#include "view_map/ViewMap.h"
#include "view_map/ViewMapBuilder.h"

static void printJSONHeader() {
    std::cout << "{\n";
}

static void printJSONFooter() {
    std::cout << "}\n";
}

static void printJSON(const std::string& key, int val, bool last = false) {
    std::cout << "  \"" << key << "\": " << val << (last ? "" : ",") << "\n";
}

static void printJSON(const std::string& key, const std::string& val, bool last = false) {
    std::cout << "  \"" << key << "\": \"" << val << "\"" << (last ? "" : ",") << "\n";
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.ply> [--json]\n";
        return 1;
    }
    bool jsonMode = (argc >= 3 && std::string(argv[2]) == "--json");

    PLYFileLoader loader(argv[1]);
    NodeGroup* root = loader.Load();
    if (!root) { std::cerr << "PLY load failed\n"; return 1; }

    WXEdgeBuilder wxBuilder;
    root->accept(wxBuilder);
    WingedEdge* we = wxBuilder.getWingedEdge();
    if (!we) { std::cerr << "Winged-edge build failed\n"; return 1; }

    real mv[4][4] = {{0}};
    real proj[4][4] = {{0}};
    int vp[4] = {0, 0, 640, 480};
    float fov = 45.0f, near = 0.1f, far = 100.0f, aspect = 640.0f/480.0f;
    float f = 1.0f / tanf(fov * (float)M_PI / 360.0f);
    mv[0][0]=1; mv[1][1]=1; mv[2][2]=1; mv[3][3]=1;
    mv[3][2] = -5.0f;
    proj[0][0] = f / aspect;
    proj[1][1] = f;
    proj[2][2] = -(far+near)/(far-near);
    proj[2][3] = -1;
    proj[3][2] = -2*far*near/(far-near);

    SilhouetteGeomEngine* sge = SilhouetteGeomEngine::getInstance();
    sge->SetViewpoint(Vec3r(0, 0, 5));
    sge->SetTransform(mv, proj, vp, 1.0f);
    sge->SetFrustum(near, far);

    FEdgeXDetector detector;
    detector.enableRidgesAndValleysFlag(true);
    detector.enableSuggestiveContours(true);
    detector.setSphereRadius(1.0f);
    detector.processShapes(*we);

    // Count faces, edges, front-facing faces
    int totalFaces = 0, totalEdges = 0, frontFacing = 0;
    int silhouette = 0, border = 0, crease = 0, ridge = 0, valley = 0, suggestive = 0;
    Vec3r cameraPos(0, 0, 5);

    for (auto sit = we->getWShapes().begin(); sit != we->getWShapes().end(); ++sit) {
        WShape* shape = *sit;
        totalFaces += shape->GetFaceList().size();

        for (auto fit = shape->GetFaceList().begin(); fit != shape->GetFaceList().end(); ++fit) {
            WXFace* face = dynamic_cast<WXFace*>(*fit);
            if (!face) continue;
            if (face->front()) ++frontFacing;
        }

        for (auto eit = shape->GetEdgeList().begin(); eit != shape->GetEdgeList().end(); ++eit) {
            WXEdge* edge = dynamic_cast<WXEdge*>(*eit);
            if (!edge) continue;
            totalEdges++;
            unsigned n = edge->nature();
            if (n & Nature::SILHOUETTE) ++silhouette;
            if (n & Nature::BORDER) ++border;
            if (n & Nature::CREASE) ++crease;
            if (n & Nature::RIDGE) ++ridge;
            if (n & Nature::VALLEY) ++valley;
            if (n & Nature::SUGGESTIVE_CONTOUR) ++suggestive;
        }
    }

    if (jsonMode) {
        std::cout << "{\n";
        std::cout << "  \"faces\": " << totalFaces << ",\n";
        std::cout << "  \"edges\": " << totalEdges << ",\n";
        std::cout << "  \"front_facing_faces\": " << frontFacing << ",\n";
        std::cout << "  \"silhouette_edges\": " << silhouette << ",\n";
        std::cout << "  \"border_edges\": " << border << ",\n";
        std::cout << "  \"crease_edges\": " << crease << ",\n";
        std::cout << "  \"ridge_edges\": " << ridge << ",\n";
        std::cout << "  \"valley_edges\": " << valley << ",\n";
        std::cout << "  \"suggestive_edges\": " << suggestive << "\n";
        std::cout << "}\n";
    } else {
        std::cerr << "faces=" << totalFaces << " edges=" << totalEdges
                  << " front_facing=" << frontFacing
                  << " silhouette=" << silhouette
                  << " border=" << border
                  << " crease=" << crease
                  << " ridge=" << ridge
                  << " valley=" << valley
                  << " suggestive=" << suggestive << "\n";
    }

    return 0;
}
