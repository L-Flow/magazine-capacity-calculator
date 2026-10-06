#pragma once

#include <QFutureWatcher>
#include <QMainWindow>
#include <QString>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <optional>
#include <atomic>
#include <memory>
#include <vector>

#include "cad/CadImporter.hpp"
#include "cad/AssemblyPackingRegion.hpp"
#include "packing/LatticePacking.hpp"
#include "packing/PackingRegion.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class OcctViewport;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(bool autoCompute = true);
    ~MainWindow() override;
    bool saveSnapshot(const QString& path);

private:
    void runLattice();
    void runSettling();
    void cancelSettling();
    void finishSettling();
    void finishAssemblyExtraction();
    void openCad();
    void beginFaceSelection();
    void beginPointSelection();
    void applyGeometrySettings();
    void handleFacesSelected(const std::vector<TopoDS_Face>& faces);
    void handlePointSelected(const gp_Pnt& seed, const gp_Pnt& entry,
                             const TopoDS_Face& face);
    void showResult(const QString& label, std::size_t count,
                    const QString& validation);
    void presentPacking(const magazine::packing::PackingRegion& region,
                        const magazine::packing::PackingResult& result);

    QDoubleSpinBox* width_{nullptr};
    QDoubleSpinBox* depth_{nullptr};
    QDoubleSpinBox* height_{nullptr};
    QDoubleSpinBox* gravityX_{nullptr};
    QDoubleSpinBox* gravityY_{nullptr};
    QDoubleSpinBox* gravityZ_{nullptr};
    QComboBox* caliber_{nullptr};
    QPushButton* latticeButton_{nullptr};
    QPushButton* settlingButton_{nullptr};
    QPushButton* cancelSettlingButton_{nullptr};
    QPushButton* openCadButton_{nullptr};
    QPushButton* selectEntryFaceButton_{nullptr};
    QPushButton* selectInteriorPointButton_{nullptr};
    QPushButton* applyGeometryButton_{nullptr};
    QLabel* cadLabel_{nullptr};
    QLabel* resultLabel_{nullptr};
    OcctViewport* viewport_{nullptr};
    TopoDS_Shape cadShape_;
    magazine::cad::CadImportResult importedCad_;
    bool hasImportedCad_{false};
    TopoDS_Face entryFace_;
    bool hasEntryFace_{false};
    std::vector<TopoDS_Face> boundaryFaces_;
    bool hasBoundaryFaces_{false};
    gp_Pnt entrySeedPoint_;
    gp_Pnt entryGatePoint_;
    bool hasEntryPoint_{false};
    std::optional<magazine::packing::PackingRegion> cadRegion_;
    std::optional<magazine::packing::PackingResult> latticeReference_;
    std::optional<magazine::cad::AssemblyPackingRegion> assemblyRegion_;
    QString cadDescription_;
    QFutureWatcher<magazine::packing::PackingResult> settlingWatcher_;
    std::shared_ptr<std::atomic_bool> settlingCancel_;
    std::optional<magazine::packing::PackingRegion> settlingRegion_;
    QFutureWatcher<magazine::cad::AssemblyPackingRegion> assemblyWatcher_;
    std::optional<gp_Dir> pendingGravity_;
};
