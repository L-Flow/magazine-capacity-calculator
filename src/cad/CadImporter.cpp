#include "cad/CadImporter.hpp"

#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#if defined(MAGAZINE_HAS_OCCT_STL)
#include <StlAPI_Reader.hxx>
#endif
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>

namespace magazine::cad {
namespace {

std::string lowercaseExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    return extension;
}

void collectShapeStats(CadImportResult& result) {
    Bnd_Box bounds;
    BRepBndLib::Add(result.shape, bounds);
    if (bounds.IsVoid()) {
        throw CadImportError("CAD model has no measurable geometry");
    }
    bounds.Get(result.minX, result.minY, result.minZ,
               result.maxX, result.maxY, result.maxZ);
    for (TopExp_Explorer it(result.shape, TopAbs_SOLID); it.More(); it.Next()) {
        ++result.solidCount;
    }
    for (TopExp_Explorer it(result.shape, TopAbs_SHELL); it.More(); it.Next()) {
        ++result.shellCount;
    }
    for (TopExp_Explorer it(result.shape, TopAbs_FACE); it.More(); it.Next()) {
        ++result.faceCount;
    }
}

TopoDS_Shape readStep(const std::filesystem::path& path) {
    STEPControl_Reader reader;
    const std::string utf8Path = path.u8string();
    const IFSelect_ReturnStatus status = reader.ReadFile(utf8Path.c_str());
    if (status != IFSelect_RetDone || reader.TransferRoots() == 0) {
        throw CadImportError("OCCT could not read STEP file: " + utf8Path);
    }
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw CadImportError("STEP file contains no transferable shape: " + utf8Path);
    }
    return shape;
}

TopoDS_Shape readStl(const std::filesystem::path& path) {
#if defined(MAGAZINE_HAS_OCCT_STL)
    StlAPI_Reader reader;
    TopoDS_Shape shape;
    const std::string utf8Path = path.u8string();
    if (!reader.Read(shape, utf8Path.c_str()) || shape.IsNull()) {
        throw CadImportError("OCCT could not read STL file: " + utf8Path);
    }
    return shape;
#else
    throw CadImportError(
        "this OCCT SDK was built without TKDESTL; use STEP or install the "
        "OCCT STL module: " + path.u8string());
#endif
}

} // namespace

CadImportResult readCad(const std::string& pathString) {
    return readCad(std::filesystem::u8path(pathString));
}

CadImportResult readCad(const std::filesystem::path& path) {
    if (!std::filesystem::is_regular_file(path)) {
        throw CadImportError("CAD file does not exist: " + path.u8string());
    }

    CadImportResult result;
    result.path = path.u8string();
    const std::string extension = lowercaseExtension(path);
    if (extension == ".step" || extension == ".stp") {
        result.format = "STEP";
        result.shape = readStep(path);
    } else if (extension == ".stl") {
        result.format = "STL";
        result.shape = readStl(path);
    } else {
        throw CadImportError("unsupported CAD extension: " + extension);
    }

    if (!BRepCheck_Analyzer(result.shape).IsValid()) {
        throw CadImportError("CAD model topology is invalid");
    }
    collectShapeStats(result);
    return result;
}

} // namespace magazine::cad
