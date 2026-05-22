// camera.h — simple perspective/orthographic camera for engraving pipeline
//
// Wraps the Freestyle SilhouetteGeomEngine view/projection setup into a
// self-contained class with position, target, up, FOV, and projection.
// Provides projectPoint() for 3D→2D and isFrontFacing() for face culling.
//
// Usage:
//   Camera cam(Vec3r(2.8, 2.0, 3.5), Vec3r(0,0,0), Vec3r(0,1,0), 35.0);
//   double x, y;
//   cam.projectPoint(someVec3, 800, 600, x, y);
//   bool front = cam.isFrontFacing(faceNormal, faceCenter);

#ifndef ENGRAVING_CAMERA_H
#define ENGRAVING_CAMERA_H

#include "geometry/Geom.h"
#include <cmath>

using Geometry::Vec3r;

struct Camera {
    Vec3r position;
    Vec3r target;
    Vec3r up;
    double fovY_degrees;
    bool orthographic = false;       // true → ortho, false → perspective

    // Derived (computed on construction)
    Vec3r forward;
    Vec3r right;
    Vec3r trueUp;

    Camera(const Vec3r& pos, const Vec3r& tgt, const Vec3r& upVec,
           double fovY = 35.0, bool ortho = false)
        : position(pos), target(tgt), up(upVec),
          fovY_degrees(fovY), orthographic(ortho)
    {
        forward = tgt - pos;
        forward.normalize();
        right = forward ^ upVec;
        right.normalize();
        trueUp = right ^ forward;
        trueUp.normalize();
    }

    // Default: front-facing camera at origin
    Camera()
        : Camera(Vec3r(2.8, 2.0, 3.5), Vec3r(0,0,0), Vec3r(0,1,0), 35.0)
    {}

    /// Project a 3D world-space point to 2D image coordinates.
    /// outX, outY are in [0, imgW] × [0, imgH].
    void projectPoint(const Vec3r& pt,
                      int imgW, int imgH,
                      double& outX, double& outY) const
    {
        Vec3r rel = pt - position;
        double x = rel * right;
        double y = rel * trueUp;
        double z = rel * forward;
        if (z < 1e-9) z = 1e-9;
        double aspect = (double)imgW / (double)imgH;
        double tanHalfFov = tan(fovY_degrees * M_PI / 360.0);
        double ndcX = x / (z * tanHalfFov * aspect);
        double ndcY = y / (z * tanHalfFov);
        outX = (ndcX + 1.0) * 0.5 * imgW;
        outY = (1.0 - (ndcY + 1.0) * 0.5) * imgH;
    }

    /// Is a face with given normal at given center visible from this camera?
    bool isFrontFacing(const Vec3r& normal, const Vec3r& center) const {
        Vec3r viewDir = position - center;
        return (normal * viewDir) > 0.0;
    }

    /// View direction FROM the camera toward a point.
    Vec3r viewDirectionAt(const Vec3r& pt) const {
        Vec3r dir = position - pt;
        dir.normalize();
        return dir;
    }
};

#endif // ENGRAVING_CAMERA_H
