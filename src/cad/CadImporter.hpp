#pragma once

#include <filesystem>
#include <string>
#include <cstddef>
#include <stdexcept>

#include <TopoDS_Shape.hxx>

namespace magazine::cad {

struct CadImportResult {
    std::string path;
    std::string format;
    std::string sourceUnit{"unknown"};
    TopoDS_Shape shape;
    double minX{0.0};
    double minY{0.0};
    double minZ{0.0};
    double maxX{0.0};
    double maxY{0.0};
    double maxZ{0.0};
    std::size_t solidCount{0};
    std::size_t shellCount{0};
    std::size_t faceCount{0};
    // Assembly exports often contain one malformed or intentionally open
    // component even though the remaining solids are usable as obstacles.
    // Keep that diagnostic separate from the import result so the assembly
    // extraction path can continue and report the limitation to the user.
    bool topologyValid{true};

    double widthMm() const { return maxX - minX; }
    double depthMm() const { return maxY - minY; }
    double heightMm() const { return maxZ - minZ; }
};

class CadImportError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

CadImportResult readCad(const std::string& path);
CadImportResult readCad(const std::filesystem::path& path);

} // namespace magazine::cad
