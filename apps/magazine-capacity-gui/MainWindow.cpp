#include "MainWindow.hpp"

#include "OcctViewport.hpp"
#include "cad/CadImporter.hpp"
#include "cad/CadPackingRegion.hpp"
#include "packing/LatticePacking.hpp"
#include "packing/QuasiStaticSettler.hpp"
#include "packing/Validation.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <gp_Dir.hxx>

#include <exception>
#include <filesystem>
#include <stdexcept>

namespace {

QDoubleSpinBox* dimensionSpin(double value) {
    auto* spin = new QDoubleSpinBox;
    spin->setRange(20.0, 2000.0);
    spin->setDecimals(1);
    spin->setSuffix(QStringLiteral(" mm"));
    spin->setValue(value);
    return spin;
}

QDoubleSpinBox* directionSpin(double value) {
    auto* spin = new QDoubleSpinBox;
    spin->setRange(-1.0e6, 1.0e6);
    spin->setDecimals(3);
    spin->setSingleStep(0.1);
    spin->setValue(value);
    return spin;
}

} // namespace

MainWindow::MainWindow(bool autoCompute) {
    setWindowTitle(QStringLiteral("弹仓静态容量计算器 0.1.3"));
    resize(1180, 760);

    auto* central = new QWidget;
    auto* root = new QHBoxLayout(central);
    auto* controls = new QVBoxLayout;
    auto* form = new QFormLayout;

    width_ = dimensionSpin(170.0);
    depth_ = dimensionSpin(100.0);
    height_ = dimensionSpin(80.0);
    gravityX_ = directionSpin(0.0);
    gravityY_ = directionSpin(-1.0);
    gravityZ_ = directionSpin(0.0);
    caliber_ = new QComboBox;
    caliber_->addItem(QStringLiteral("17 mm"), 17.0);
    caliber_->addItem(QStringLiteral("42 mm"), 42.0);

    form->addRow(QStringLiteral("内部宽度"), width_);
    form->addRow(QStringLiteral("内部深度"), depth_);
    form->addRow(QStringLiteral("内部高度"), height_);
    form->addRow(QStringLiteral("弹丸直径"), caliber_);
    form->addRow(QStringLiteral("重力 X"), gravityX_);
    form->addRow(QStringLiteral("重力 Y"), gravityY_);
    form->addRow(QStringLiteral("重力 Z"), gravityZ_);

    latticeButton_ = new QPushButton(QStringLiteral("计算 FCC/HCP 理想参考"));
    settlingButton_ = new QPushButton(QStringLiteral("运行无摩擦准静态沉降"));
    openCadButton_ = new QPushButton(QStringLiteral("打开 STEP / STL"));
    selectEntryFaceButton_ =
        new QPushButton(QStringLiteral("选择弹丸进入面（左键点面）"));
    applyGeometryButton_ =
        new QPushButton(QStringLiteral("应用入口面和重力方向"));
    cadLabel_ = new QLabel(QStringLiteral("未加载 CAD，当前使用参数化长方体"));
    cadLabel_->setWordWrap(true);
    resultLabel_ = new QLabel(QStringLiteral("选择一种算法开始计算"));
    resultLabel_->setWordWrap(true);
    resultLabel_->setMinimumWidth(270);

    controls->addLayout(form);
    controls->addWidget(openCadButton_);
    controls->addWidget(selectEntryFaceButton_);
    controls->addWidget(applyGeometryButton_);
    controls->addWidget(cadLabel_);
    controls->addWidget(latticeButton_);
    controls->addWidget(settlingButton_);
    controls->addWidget(resultLabel_);
    controls->addStretch();

    viewport_ = new OcctViewport;
    root->addLayout(controls);
    root->addWidget(viewport_, 1);
    setCentralWidget(central);

    viewport_->setFaceSelectionCallback(
        [this](const TopoDS_Face& face) { handleFaceSelected(face); });

    if (!autoCompute) {
        const magazine::packing::AxisAlignedBox previewBox{
            width_->value(), depth_->value(), height_->value()};
        viewport_->setPacking(
            previewBox,
            magazine::packing::PackingResult{"smoke preview", 0.0, {}, 0, 0});
    }

    connect(latticeButton_, &QPushButton::clicked, this,
            [this] { runLattice(); });
    connect(settlingButton_, &QPushButton::clicked, this,
            [this] { runSettling(); });
    connect(openCadButton_, &QPushButton::clicked, this,
            [this] { openCad(); });
    connect(selectEntryFaceButton_, &QPushButton::clicked, this,
            [this] { beginFaceSelection(); });
    connect(applyGeometryButton_, &QPushButton::clicked, this,
            [this] { applyGeometrySettings(); });

    if (autoCompute) {
        QTimer::singleShot(0, this, [this] { runLattice(); });
    }
}

bool MainWindow::saveSnapshot(const QString& path) {
    return viewport_ != nullptr && viewport_->saveSnapshot(path);
}

void MainWindow::runLattice() {
    try {
        const magazine::packing::AxisAlignedBox inputBox{
            width_->value(), depth_->value(), height_->value()};
        const double radius = caliber_->currentData().toDouble() / 2.0;
        const auto fallback = magazine::packing::boxPackingRegion(inputBox);
        const auto& region = cadRegion_.has_value() ? *cadRegion_ : fallback;
        magazine::packing::LatticeOptions latticeOptions;
        if (cadRegion_.has_value()) {
            // CAD concave-pocket fallback classification is substantially
            // more expensive than the box fast path. Two phase offsets keep
            // the interactive result responsive while still checking both
            // FCC and HCP arrangements.
            latticeOptions.phaseDivisions = 2;
        }
        const auto result = magazine::packing::packBestFccOrHcp(
            region, radius, latticeOptions);
        const auto validation = magazine::packing::validatePacking(region, result);
        presentPacking(region, result);
        const QString method = !cadRegion_.has_value()
            ? QString::fromStdString(result.method)
             : QStringLiteral("%1（CAD 真实内腔，已按用户重力方向对齐）")
                  .arg(QString::fromStdString(result.method));
        showResult(method, result.centers.size(),
                   QString::fromStdString(validation.message));
    } catch (const std::exception& error) {
        resultLabel_->setText(QStringLiteral("计算失败：%1")
                                  .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::runSettling() {
    try {
        const magazine::packing::AxisAlignedBox inputBox{
            width_->value(), depth_->value(), height_->value()};
        const double radius = caliber_->currentData().toDouble() / 2.0;
        const auto fallback = magazine::packing::boxPackingRegion(inputBox);
        const auto& region = cadRegion_.has_value() ? *cadRegion_ : fallback;
        magazine::packing::SettlingOptions options;
        const auto result =
            magazine::packing::settleWithoutFriction(region, radius, options);
        const auto validation =
            magazine::packing::validatePacking(region, result, 0.15);
        presentPacking(region, result);
        const QString method = !cadRegion_.has_value()
            ? QStringLiteral("无摩擦准静态沉降")
             : QStringLiteral("无摩擦准静态沉降（CAD 真实内腔，已按用户重力方向对齐）");
        showResult(method, result.centers.size(),
                   QString::fromStdString(validation.message));
    } catch (const std::exception& error) {
        resultLabel_->setText(QStringLiteral("计算失败：%1")
                                  .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::openCad() {
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("选择弹仓 CAD 模型"), QString(),
        QStringLiteral("CAD files (*.step *.stp *.stl);;All files (*.*)"));
    if (path.isEmpty()) return;

    try {
        const auto imported = magazine::cad::readCad(
            std::filesystem::path(path.toStdWString()));
        if (imported.widthMm() <= 0.0 || imported.depthMm() <= 0.0 ||
            imported.heightMm() <= 0.0) {
            throw std::runtime_error("CAD 包围盒尺寸无效");
        }
        importedCad_ = imported;
        hasImportedCad_ = true;
        hasEntryFace_ = false;
        entryFace_.Nullify();
        cadRegion_.reset();
        cadShape_ = imported.shape;
        cadDescription_ = QStringLiteral("%1\n内腔坐标尺寸：%2 x %3 x %4 mm\n实体 %5，壳 %6，面 %7\n已按 Y 方向垂直落入；球体完整位于 CAD 内腔后计数")
            .arg(QString::fromStdString(imported.path))
            .arg(imported.widthMm(), 0, 'f', 2)
            .arg(imported.depthMm(), 0, 'f', 2)
            .arg(imported.heightMm(), 0, 'f', 2)
            .arg(static_cast<qulonglong>(imported.solidCount))
            .arg(static_cast<qulonglong>(imported.shellCount))
            .arg(static_cast<qulonglong>(imported.faceCount));
        cadDescription_ = QStringLiteral("%1\n源模型包围盒：%2 x %3 x %4 mm\n实体 %5，壳 %6，面 %7\n请在右侧点击入口面，再应用重力方向")
            .arg(QString::fromStdString(imported.path))
            .arg(imported.widthMm(), 0, 'f', 2)
            .arg(imported.depthMm(), 0, 'f', 2)
            .arg(imported.heightMm(), 0, 'f', 2)
            .arg(static_cast<qulonglong>(imported.solidCount))
            .arg(static_cast<qulonglong>(imported.shellCount))
            .arg(static_cast<qulonglong>(imported.faceCount));
        cadLabel_->setText(cadDescription_);
        viewport_->clearPacking();
        viewport_->setCadShape(cadShape_);
        viewport_->setFaceSelectionEnabled(true);
    } catch (const std::exception& error) {
        cadLabel_->setText(QStringLiteral("CAD 导入失败：%1")
                               .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::beginFaceSelection() {
    if (!hasImportedCad_) {
        cadLabel_->setText(QStringLiteral("请先打开 STEP / STL 模型"));
        return;
    }
    cadRegion_.reset();
    hasEntryFace_ = false;
    entryFace_.Nullify();
    cadShape_ = importedCad_.shape;
    viewport_->clearPacking();
    viewport_->setCadShape(cadShape_);
    viewport_->setFaceSelectionEnabled(true);
    cadLabel_->setText(cadDescription_ +
                       QStringLiteral("\n正在选择入口面：请在模型上左键点击任意面"));
}

void MainWindow::handleFaceSelected(const TopoDS_Face& face) {
    if (face.IsNull()) return;
    entryFace_ = face;
    hasEntryFace_ = true;
    cadLabel_->setText(cadDescription_ +
                       QStringLiteral("\n入口面已选择，请点击“应用入口面和重力方向”"));
}

void MainWindow::applyGeometrySettings() {
    if (!hasImportedCad_) {
        cadLabel_->setText(QStringLiteral("请先打开 STEP / STL 模型"));
        return;
    }
    if (!hasEntryFace_) {
        cadLabel_->setText(QStringLiteral("请先在右侧三维模型上选择入口面"));
        return;
    }
    const double gx = gravityX_->value();
    const double gy = gravityY_->value();
    const double gz = gravityZ_->value();
    if (gx * gx + gy * gy + gz * gz < 1.0e-12) {
        cadLabel_->setText(QStringLiteral("重力方向不能是零向量"));
        return;
    }
    try {
        const gp_Dir gravity(gx, gy, gz);
        const auto aligned = magazine::cad::makeGravityAlignedPackingRegion(
            importedCad_, gravity, entryFace_);
        cadRegion_ = aligned.region;
        cadShape_ = aligned.displayShape;
        width_->setValue(aligned.region.bounds.widthMm);
        depth_->setValue(aligned.region.bounds.depthMm);
        height_->setValue(aligned.region.bounds.heightMm);
        viewport_->setFaceSelectionEnabled(false);
        viewport_->setCadShape(cadShape_);
        cadLabel_->setText(
            QStringLiteral("入口面已应用\n重力方向：(%1, %2, %3)\n已转换到局部重力坐标系：局部 -Z 为重力")
                .arg(gx, 0, 'f', 3)
                .arg(gy, 0, 'f', 3)
                .arg(gz, 0, 'f', 3));
        runLattice();
    } catch (const std::exception& error) {
        cadLabel_->setText(QStringLiteral("入口面/重力应用失败：%1")
                               .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::presentPacking(
    const magazine::packing::PackingRegion& region,
    const magazine::packing::PackingResult& result) {
    auto displayResult = result;
    viewport_->setPacking(region.bounds, displayResult);
}

void MainWindow::showResult(const QString& label, std::size_t count,
                            const QString& validation) {
    resultLabel_->setText(
        QStringLiteral("%1\n数量：%2 发\n校验：%3")
            .arg(label)
            .arg(static_cast<qulonglong>(count))
            .arg(validation));
}
