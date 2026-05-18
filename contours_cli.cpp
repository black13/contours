#include <cstdio>
#include <vector>
#include <string>
#include "tess_RifFilter/refineContour.h"
#include "tess_RifFilter/subdiv.h"
#include "tess_RifFilter/cameraModel.h"
#include "hbr/mesh.h"
#include "hbr/catmark.h"

using namespace OpenSubdiv;

// ===================================================================
//  Debug dump utilities — callable from lldb:
//    ? exec dumpMesh(mesh, 5)
//    ? exec dumpFace(face, 0)
//    ? exec dumpCatmarkMesh(surface, 10)
// ===================================================================

static void dumpVertex(MeshVertex* v, int idx)
{
    auto& d = v->GetData();
    printf("  v%03d: pos=(%.6f,%.6f,%.6f) N=(%.4f,%.4f,%.4f) ndotv=%.6g facing=%d radial=%d\n",
           idx, (double)d.pos[0], (double)d.pos[1], (double)d.pos[2],
           (double)d.normal[0], (double)d.normal[1], (double)d.normal[2],
           (double)d.ndotv, d.facing, d.Radial());
    if (d.Radial())
        printf("         radialOrg[0]=%p [1]=%p\n", (void*)d.radialOrg[0], (void*)d.radialOrg[1]);
    printf("         k1=%.6g k2=%.6g cusp=%d extr=%d isShift=%d age=%d\n",
           (double)d.k1, (double)d.k2, d.cusp, d.extraordinary, d.isShifted, d.age);
}

static void dumpFace(MeshFace* face, int idx)
{
    int nv = face->GetNumVertices();
    printf("Face %03d [%d verts]: ", idx, nv);
    for (int vi = 0; vi < nv; vi++) {
        auto& d = face->GetVertex(vi)->GetData();
        char s = d.ndotv > 1e-8 ? '+' : d.ndotv < -1e-8 ? '-' : '0';
        printf("%c(%.4g) ", s, (double)d.ndotv);
    }
    printf("\n");
    for (int vi = 0; vi < nv; vi++)
        dumpVertex(face->GetVertex(vi), vi);
}

static void dumpMesh(Mesh* mesh, int maxFaces = 99999)
{
    printf("\n=== MESH DUMP ===\n");
    std::list<MeshFace*> facelist;
    mesh->GetFaces(std::back_inserter(facelist));
    printf("Faces: %lu  (showing first %d)\n", facelist.size(), maxFaces);
    int fi = 0;
    for (auto it = facelist.begin(); it != facelist.end() && fi < maxFaces; ++it, ++fi)
        dumpFace(*it, fi);
    printf("=== END MESH DUMP ===\n\n");
}

// --- Catmark (control mesh) dumps ---

static void dumpCatmarkVertex(CatmarkVertex* v, int idx)
{
    vec3 pos = v->GetData().GetPos();
    printf("  cv%03d: pos=(%.4f,%.4f,%.4f) boundary=%d valence=%d\n",
           idx, (double)pos[0], (double)pos[1], (double)pos[2],
           v->OnBoundary(), v->GetValence());
}

static void dumpCatmarkFace(CatmarkFace* face, int idx)
{
    int nv = face->GetNumVertices();
    printf("CFace %03d [%d verts] depth=%d hasLimit=%d: ", idx, nv,
           face->GetDepth(), face->HasLimit());
    for (int vi = 0; vi < nv; vi++)
        printf("cv%d ", face->GetVertex(vi)->GetID());
    printf("\n");
    for (int vi = 0; vi < nv; vi++)
        dumpCatmarkVertex(face->GetVertex(vi), vi);
}

static void dumpCatmarkMesh(CatmarkMesh* cm, int maxFaces = 99999)
{
    printf("\n=== CATMARK MESH DUMP ===\n");
    printf("Verts: %d  Faces: %d  Coarse: %d\n",
           cm->GetNumVertices(), cm->GetNumFaces(), cm->GetNumCoarseFaces());
    std::list<CatmarkFace*> facelist;
    cm->GetFaces(std::back_inserter(facelist));
    printf("Faces: %lu  (showing first %d)\n", facelist.size(), maxFaces);
    int fi = 0;
    for (auto it = facelist.begin(); it != facelist.end() && fi < maxFaces; ++it, ++fi)
        dumpCatmarkFace(*it, fi);
    printf("=== END CATMARK DUMP ===\n\n");
}

// ===================================================================

// Build a simple combined view-projection camera matrix
static CameraModel makeSimpleCamera(const vec3& eye, float width, float height)
{
    // Simple matrix: translate by -eye, then simple perspective
    mat4 cm = mat4::identity();
    cm(0,3) = -eye[0];
    cm(1,3) = -eye[1];
    cm(2,3) = -eye[2];

    // Near/far/viewport — generous values for testing
    // top=1, bottom=-1 (NDC convention: screen top is +1, bottom is -1)
    return CameraModel(cm, 0.01f, 100.0f, -1.0f, 1.0f, 1.0f, -1.0f,
                       width, height, 1.0f, eye);
}

CatmarkMesh* buildCube(float size)
{
    static HbrCatmarkSubdivision<Vertex> catmark;
    CatmarkMesh* mesh = new CatmarkMesh(&catmark);
    mesh->SetInterpolateBoundaryMethod(CatmarkMesh::k_InterpolateBoundaryEdgeOnly);

    float h = size * 0.5f;
    // 8 vertices
    float corners[8][3] = {
        {-h,-h,-h}, { h,-h,-h}, { h, h,-h}, {-h, h,-h},
        {-h,-h, h}, { h,-h, h}, { h, h, h}, {-h, h, h}
    };
    for (int i = 0; i < 8; i++) {
        Vertex v(corners[i][0], corners[i][1], corners[i][2]);
        mesh->NewVertex(i, v);
    }

    // 6 quad faces (CCW winding), number of vertices per face = 4
    int faces[6][4] = {
        {0,3,2,1}, // -Z
        {4,5,6,7}, // +Z
        {0,1,5,4}, // -Y
        {2,3,7,6}, // +Y
        {0,4,7,3}, // -X
        {1,2,6,5}  // +X
    };
    for (int i = 0; i < 6; i++) {
        mesh->NewFace(4, faces[i], i);
    }
    mesh->Finish();
    return mesh;
}

CatmarkMesh* buildCylinder(float radius, float height, int segments)
{
    static HbrCatmarkSubdivision<Vertex> catmark;
    CatmarkMesh* mesh = new CatmarkMesh(&catmark);
    mesh->SetInterpolateBoundaryMethod(CatmarkMesh::k_InterpolateBoundaryEdgeOnly);

    float h = height * 0.5f;
    int nv = segments * 2 + 2; // top + bottom rings + 2 poles (approximated with val-3 fans)
    // Simplified: just use a 2-ring quad cylinder
    mesh->Finish();
    return mesh;
}

CatmarkMesh* buildTetrahedron(float size)
{
    static HbrCatmarkSubdivision<Vertex> catmark;
    CatmarkMesh* mesh = new CatmarkMesh(&catmark);
    mesh->SetInterpolateBoundaryMethod(CatmarkMesh::k_InterpolateBoundaryEdgeOnly);

    float s = size / sqrtf(2.0f);
    float v[4][3] = {{ s, s, s}, { s,-s,-s}, {-s, s,-s}, {-s,-s, s}};
    for (int i = 0; i < 4; i++)
        mesh->NewVertex(i, Vertex(v[i][0], v[i][1], v[i][2]));
    int f[4][3] = {{0,2,1}, {0,1,3}, {0,3,2}, {1,2,3}};
    for (int i = 0; i < 4; i++)
        mesh->NewFace(3, f[i], i);
    mesh->Finish();
    return mesh;
}

CatmarkMesh* buildArch(float width, float height, float depth, int segments)
{
    return buildCube(std::max(width, height));
}

CatmarkMesh* buildIcosahedron()
{
    const float phi = (1.0f + sqrtf(5.0f)) * 0.5f;
    static HbrCatmarkSubdivision<Vertex> catmark;
    CatmarkMesh* mesh = new CatmarkMesh(&catmark);
    mesh->SetInterpolateBoundaryMethod(CatmarkMesh::k_InterpolateBoundaryEdgeOnly);

    // 12 vertices of a regular icosahedron
    float verts[12][3] = {
        { 0,  1,  phi},{ 0, -1,  phi},{ 0,  1, -phi},{ 0, -1, -phi},
        { 1,  phi, 0},{-1,  phi, 0},{ 1, -phi, 0},{-1, -phi, 0},
        { phi, 0,  1},{-phi, 0,  1},{ phi, 0, -1},{-phi, 0, -1}
    };
    for (int i = 0; i < 12; i++) {
        Vertex v(verts[i][0], verts[i][1], verts[i][2]);
        mesh->NewVertex(i, v);
    }

    // 20 triangular faces
    int faces[20][3] = {
        {0,1,4}, {0,4,5}, {0,5,8}, {0,8,9}, {0,9,1},
        {1,9,7}, {1,7,6}, {1,6,4},
        {2,3,6}, {2,6,7}, {2,7,11},{2,11,10},{2,10,3},
        {3,10,4},{3,4,6},
        {5,4,10},{5,10,8},
        {8,10,11},{8,11,9},
        {9,11,7}
    };
    for (int i = 0; i < 20; i++) {
        mesh->NewFace(3, faces[i], i);
    }
    mesh->Finish();
    return mesh;
}

void printUsage(const char* prog)
{
    std::cout << "Usage: " << prog << " <primitive> [options]\n"
              << "  primitives: tetra, cube, icosphere\n"
              << "  options:\n"
              << "    -subd N      subdivision level (default: 3)\n"
              << "    -size S      size (default: 2.0)\n"
              << "    -cam X Y Z   camera position (default: 0 0 5)\n"
              << "    -o FILE      output file prefix (default: output)\n";
}

int main(int argc, char** argv)
{
    std::cerr << "contours_cli starting\n";

    std::string primitive = "cube";
    float size = 2.0f;
    int subdLevel = 3;
    vec3 cameraPos(0, 0, 5);
    std::string outPrefix = "output";
    bool checkMode = false;

    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "-subd" && i+1 < argc) subdLevel = atoi(argv[++i]);
        else if (arg == "-size" && i+1 < argc) size = (float)atof(argv[++i]);
        else if (arg == "-cam" && i+3 < argc) {
            cameraPos[0] = (float)atof(argv[++i]);
            cameraPos[1] = (float)atof(argv[++i]);
            cameraPos[2] = (float)atof(argv[++i]);
        }
        else if (arg == "-o" && i+1 < argc) outPrefix = argv[++i];
        else if (arg == "--check") { checkMode = true; std::cerr << "checkMode ON\n"; }
        else if (arg == "-h" || arg == "--help") { printUsage(argv[0]); return 0; }
        else { primitive = arg; }
    }

    CatmarkMesh* surface = nullptr;
    if (primitive == "tetra")
        surface = buildTetrahedron(size);
    else if (primitive == "cube" || primitive == "arch") {
        std::cerr << "building cube...\n";
        surface = buildCube(size);
        std::cerr << "cube built\n";
    }
    else if (primitive == "icosphere")
        surface = buildIcosahedron();
    else {
        std::cerr << "Unknown primitive: " << primitive << "\n";
        printUsage(argv[0]);
        return 1;
    }

    if (!surface) {
        std::cerr << "Failed to build surface\n";
        return 1;
    }
    
    dumpCatmarkMesh(surface);
    
    // Set up camera
    float imgW = 640.0f, imgH = 480.0f;
    CameraModel camera = makeSimpleCamera(cameraPos, imgW, imgH);

    std::cout << "Surface built: " << surface->GetNumVertices() << " verts, "
              << surface->GetNumFaces() << " faces\n";
    std::cout << "Subdividing to level " << subdLevel << "...\n";

    // Sample surface into working mesh (triangles=true to match SavePLYFile expectation)
    Mesh* mesh = SurfaceToMesh(surface, subdLevel, camera, true);
    if (!mesh) {
        std::cerr << "SurfaceToMesh failed\n";
        return 1;
    }
    std::cout << "Sampled mesh: " << mesh->GetNumVertices() << " verts, "
              << mesh->GetNumFaces() << " faces\n";

    // Save pre-refinement mesh
    SavePLYFile(mesh, (outPrefix + "_initial").c_str(), 0);

    // Run contour refinement
    std::cout << "Refining contours...\n";
    RefineContourRadial(mesh, cameraPos, true, EVERYTHING);
    std::cout << "Refined mesh: " << mesh->GetNumVertices() << " verts, "
              << mesh->GetNumFaces() << " faces\n";

    // Save result
    SavePLYFile(mesh, (outPrefix + "_refined").c_str(), 0);

    // Stats
    int numInconsistent, numStrongInconsistent, numNonRadial, numIncContour, numIncRadial;
    ComputeConsistencyStats(mesh, cameraPos,
                            numInconsistent, numStrongInconsistent,
                            numNonRadial, numIncContour, numIncRadial);
    std::cout << "Consistency stats:\n"
              << "  Inconsistent: " << numInconsistent << "\n"
              << "  Strong inconsistent: " << numStrongInconsistent << "\n"
              << "  Non-radial: " << numNonRadial << "\n"
              << "  Inconsistent contour: " << numIncContour << "\n"
              << "  Inconsistent radial: " << numIncRadial << "\n";

    // ---- Per-face invariant walk (check mode) ----
    if (checkMode) {
        std::cerr << "PER-FACE CHECK STARTING\n";
        int allSame = 0, radialOk = 0, violations = 0;
        int allFront = 0, allBack = 0;
        int contourVertCount = 0;
        int fi = 0;

        // Use iterator pattern like ComputeConsistencyStats does
        std::list<MeshFace*> facelist;
        mesh->GetFaces(std::back_inserter(facelist));
        std::cerr << "Mesh has " << facelist.size() << " faces\n";

        for (auto it = facelist.begin(); it != facelist.end(); ++it) {
            MeshFace* face = *it;
            int nv = face->GetNumVertices();

            // Get ndotv sign for each vertex
            char signs[8]; // max 7-vertex face
            int nPos = 0, nNeg = 0, nZero = 0;
            for (int vi = 0; vi < nv && vi < 8; vi++) {
                MeshVertex* v = face->GetVertex(vi);
                real ndotv = v->GetData().ndotv;
                // Use same threshold as the algorithm: CONTOUR_THRESHOLD = 1e-8
                if (ndotv > 1e-8)             { signs[vi] = '+'; nPos++; }
                else if (ndotv < -1e-8) { signs[vi] = '-'; nNeg++; }
                else                     { signs[vi] = '0'; nZero++; }
            }

            // Classify
            enum { ALL_SAME, RADIAL, VIOLATION } result;
            if (nPos == nv)       { result = ALL_SAME; allFront++; allSame++; }
            else if (nNeg == nv)  { result = ALL_SAME; allBack++;  allSame++; }
            else if (nZero == 1 && (nPos == nv-1 || nNeg == nv-1)) {
                // Check radial: exactly one contour vertex, rest same sign
                // Find the contour vertex and verify radialOrg
                result = RADIAL;
                for (int vi = 0; vi < nv; vi++) {
                    if (signs[vi] == '0') {
                        MeshVertex* cv = face->GetVertex(vi);
                        if (cv->GetData().Radial()) {
                            radialOk++;
                            contourVertCount++;
                        } else {
                            result = VIOLATION; // contour vertex not radial
                        }
                    }
                }
                if (result == RADIAL) radialOk--;
            }
            else {
                result = VIOLATION;
            }

            if (result == VIOLATION) {
                violations++;
                std::cerr << "VIOLATION face " << fi << ": ";
                for (int vi = 0; vi < nv; vi++)
                    std::cerr << signs[vi];
                std::cerr << " (ndotv:";
                for (int vi = 0; vi < nv; vi++)
                    std::cerr << " " << face->GetVertex(vi)->GetData().ndotv;
                std::cerr << ")\n";
            }

            if (checkMode && (fi < 20 || result == VIOLATION)) {
                std::cerr << "  Face " << fi << " [" << nv << "v]: ";
                for (int vi = 0; vi < nv; vi++) std::cerr << signs[vi];
                std::cerr << " -> " << (result==ALL_SAME?"ALL_SAME":result==RADIAL?"RADIAL":"VIOLATION") << "\n";
            }
            fi++;
        }
        std::cerr << "\nSUMMARY: " << allFront << " front, " << allBack << " back, "
                  << radialOk << " radial, " << violations << " violations\n";
        std::cerr << "PER-FACE CHECK DONE\n";
    }

    std::cerr << "Wrote " << outPrefix << "_initial.0.ply and "
              << outPrefix << "_refined.0.ply\n";
    return 0;
}
