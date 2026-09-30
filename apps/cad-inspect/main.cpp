#include "cad/CadImporter.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

const char* surfaceTypeName(GeomAbs_SurfaceType type) {
    switch (type) {
    case GeomAbs_Plane: return "plane";
    case GeomAbs_Cylinder: return "cylinder";
    case GeomAbs_Cone: return "cone";
    case GeomAbs_Sphere: return "sphere";
    case GeomAbs_Torus: return "torus";
    case GeomAbs_BezierSurface: return "bezier";
    case GeomAbs_BSplineSurface: return "bspline";
    case GeomAbs_SurfaceOfRevolution: return "revolution";
    case GeomAbs_SurfaceOfExtrusion: return "extrusion";
    case GeomAbs_OffsetSurface: return "offset";
    default: return "other";
    }
}

void printDirection(const gp_Dir& direction) {
    std::cout << '(' << direction.X() << ", " << direction.Y() << ", "
              << direction.Z() << ')';
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc != 2) {
        std::cerr << "usage: cad-inspect <model.step>\n";
        return EXIT_FAILURE;
    }

    try {
        const auto model = magazine::cad::readCad(std::filesystem::path(argv[1]));
        GProp_GProps volumeProperties;
        BRepGProp::VolumeProperties(model.shape, volumeProperties);
        const gp_Pnt center = volumeProperties.CentreOfMass();

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "format: " << model.format << '\n';
        std::cout << "bounds-mm: [" << model.minX << ", " << model.minY
                  << ", " << model.minZ << "] - [" << model.maxX << ", "
                  << model.maxY << ", " << model.maxZ << "]\n";
        std::cout << "solids: " << model.solidCount
                  << ", shells: " << model.shellCount
                  << ", faces: " << model.faceCount << '\n';
        std::cout << "volume-mm3: " << volumeProperties.Mass() << '\n';
        std::cout << "center-mm: (" << center.X() << ", " << center.Y()
                  << ", " << center.Z() << ")\n";

        std::size_t faceIndex = 0;
        for (TopExp_Explorer it(model.shape, TopAbs_FACE); it.More(); it.Next()) {
            const TopoDS_Face face = TopoDS::Face(it.Current());
            BRepAdaptor_Surface surface(face, true);
            GProp_GProps areaProperties;
            BRepGProp::SurfaceProperties(face, areaProperties);
            const gp_Pnt faceCenter = areaProperties.CentreOfMass();
            std::cout << "face " << ++faceIndex << ": "
                      << surfaceTypeName(surface.GetType())
                      << ", area-mm2=" << areaProperties.Mass()
                      << ", center=(" << faceCenter.X() << ", "
                      << faceCenter.Y() << ", " << faceCenter.Z() << ')';
            if (surface.GetType() == GeomAbs_Plane) {
                gp_Dir normal = surface.Plane().Axis().Direction();
                if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
                std::cout << ", outward-normal=";
                printDirection(normal);
            } else if (surface.GetType() == GeomAbs_Cylinder) {
                std::cout << ", radius-mm=" << surface.Cylinder().Radius()
                          << ", axis=";
                printDirection(surface.Cylinder().Axis().Direction());
            }
            std::cout << '\n';
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "cad-inspect failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
