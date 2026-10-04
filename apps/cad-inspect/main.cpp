#include "cad/CadImporter.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
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
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: cad-inspect <model.step> [--solids] [--faces]\n";
        return EXIT_FAILURE;
    }

    bool printSolids = false;
    bool printFaces = false;
    for (int index = 2; index < argc; ++index) {
        const std::wstring option(argv[index]);
        if (option == L"--solids") printSolids = true;
        else if (option == L"--faces") printFaces = true;
        else {
            std::cerr << "unknown option\n";
            return EXIT_FAILURE;
        }
    }

    try {
        const auto model = magazine::cad::readCad(std::filesystem::path(argv[1]));
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "format: " << model.format << '\n';
        std::cout << "bounds-mm: [" << model.minX << ", " << model.minY
                  << ", " << model.minZ << "] - [" << model.maxX << ", "
                  << model.maxY << ", " << model.maxZ << "]\n";
        std::cout << "solids: " << model.solidCount
                  << ", shells: " << model.shellCount
                  << ", faces: " << model.faceCount
                  << ", topology-valid: " << (model.topologyValid ? "yes" : "no")
                  << '\n';
        if (model.topologyValid && model.solidCount == 1) {
            GProp_GProps volumeProperties;
            BRepGProp::VolumeProperties(model.shape, volumeProperties);
            const gp_Pnt center = volumeProperties.CentreOfMass();
            std::cout << "volume-mm3: " << volumeProperties.Mass() << '\n';
            std::cout << "center-mm: (" << center.X() << ", " << center.Y()
                      << ", " << center.Z() << ")\n";
        } else {
            std::cout << "volume-mm3: unavailable for assembly or invalid topology\n";
        }

        std::size_t faceIndex = 0;
        const bool canPrintFaces = printFaces ||
                                   (model.topologyValid && model.faceCount < 5000);
        if (!canPrintFaces) {
            std::cout << "face details: skipped for large or invalid assembly; "
                         "pass --faces to inspect them\n";
        }
        for (TopExp_Explorer it(model.shape, TopAbs_FACE);
             canPrintFaces && it.More(); it.Next()) {
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

        if (printSolids) {
            std::size_t solidIndex = 0;
            for (TopExp_Explorer it(model.shape, TopAbs_SOLID);
                 it.More(); it.Next()) {
                const TopoDS_Solid solid = TopoDS::Solid(it.Current());
                Bnd_Box bounds;
                BRepBndLib::Add(solid, bounds);
                if (bounds.IsVoid()) continue;
                double minX = 0.0;
                double minY = 0.0;
                double minZ = 0.0;
                double maxX = 0.0;
                double maxY = 0.0;
                double maxZ = 0.0;
                bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
                GProp_GProps volumeProperties;
                BRepGProp::VolumeProperties(solid, volumeProperties);
                const gp_Pnt center = volumeProperties.CentreOfMass();
                std::cout << "solid " << solidIndex++
                          << ": volume-mm3=" << volumeProperties.Mass()
                          << ", center=(" << center.X() << ", " << center.Y()
                          << ", " << center.Z() << ")"
                          << ", bounds=[" << minX << ", " << minY << ", "
                          << minZ << "]-[" << maxX << ", " << maxY << ", "
                          << maxZ << "]\n";
            }
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "cad-inspect failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
