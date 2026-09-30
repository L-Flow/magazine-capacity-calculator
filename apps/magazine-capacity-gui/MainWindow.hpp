#pragma once

#include <QMainWindow>
#include <QString>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <optional>

#include "cad/CadImporter.hpp"
#include "packing/PackingRegion.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class OcctViewport;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(bool autoCompute = true);
    bool saveSnapshot(const QString& path);

private:
    void runLattice();
    void runSettling();
    void openCad();
    void beginFaceSelection();
    void applyGeometrySettings();
    void handleFaceSelected(const TopoDS_Face& face);
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
    QPushButton* openCadButton_{nullptr};
    QPushButton* selectEntryFaceButton_{nullptr};
    QPushButton* applyGeometryButton_{nullptr};
    QLabel* cadLabel_{nullptr};
    QLabel* resultLabel_{nullptr};
    OcctViewport* viewport_{nullptr};
    TopoDS_Shape cadShape_;
    magazine::cad::CadImportResult importedCad_;
    bool hasImportedCad_{false};
    TopoDS_Face entryFace_;
    bool hasEntryFace_{false};
    std::optional<magazine::packing::PackingRegion> cadRegion_;
    QString cadDescription_;
};

