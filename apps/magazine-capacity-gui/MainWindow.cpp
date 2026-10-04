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
#include <QtConcurrent/QtConcurrentRun>

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

QString explainAssemblyExtractionFailure(const QString& error) {
    const QString lower = error.toLower();
    QString advice;
    if (lower.contains(QStringLiteral("could not find free space"))) {
        advice = QStringLiteral(
            "没有找到用户指定点所在的自由空间。请点击“点击高亮内壁生成内部点（多实体必选）”，"
            "在目标腔体的黄色高亮内壁上单击，红点由该面自动生成。\n"
            "当前边界面仍保留，可重新指定内部点后重试。");
    } else if (lower.contains(QStringLiteral("do not expose a measurable free side"))) {
        advice = QStringLiteral(
            "选中面两侧都被判定为实体，或该面拓扑无可识别自由侧。"
            "请改选弹仓内壁，而不是装配体外表面或零件接触面。\n"
            "当前选择仍保留，可继续 Ctrl+左键调整。");
    } else if (lower.contains(QStringLiteral("exceeded the cell limit")) ||
               lower.contains(QStringLiteral("bounding box is too large"))) {
        advice = QStringLiteral(
            "选面包络范围过大，网格数量超过上限。请删除远离弹仓的面，"
            "只保留围成目标腔体的相邻内壁面，再重新确认。");
    } else if (lower.contains(QStringLiteral("no measurable solid obstacles"))) {
        advice = QStringLiteral(
            "选面附近没有可用的装配体实体。请确认已打开完整 STEP 装配体，"
            "并选择实体内侧的弹仓壁面。");
    } else {
        advice = QStringLiteral(
            "请确认选择的是目标弹仓内壁面，并在黄色高亮内壁上生成内部点；"
            "边界面只用于限制范围，内部点用于消除多个连通空间的歧义。\n"
            "当前选择仍保留，可重新指定内部点后重试。");
    }
    return QStringLiteral("装配体空间提取失败\n原因：%1\n建议：%2")
        .arg(error, advice);
}

} // namespace

MainWindow::MainWindow(bool autoCompute) {
    setWindowTitle(QStringLiteral("弹仓静态容量计算器 0.2.8 v13 沉降加速版"));
    resize(1180, 760);

    auto* central = new QWidget;
    auto* root = new QHBoxLayout(central);
    auto* controls = new QVBoxLayout;
    auto* form = new QFormLayout;

    width_ = dimensionSpin(170.0);
    depth_ = dimensionSpin(100.0);
    height_ = dimensionSpin(80.0);
    // The supplied vehicle STEP uses global Z as the height axis.  Users can
    // still override this for another assembly orientation.
    gravityX_ = directionSpin(0.0);
    gravityY_ = directionSpin(0.0);
    gravityZ_ = directionSpin(-1.0);
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
    cancelSettlingButton_ = new QPushButton(QStringLiteral("停止准静态计算"));
    cancelSettlingButton_->setEnabled(false);
    openCadButton_ = new QPushButton(QStringLiteral("打开 STEP / STL"));
    selectEntryFaceButton_ =
        new QPushButton(QStringLiteral("选择弹仓边界面（Ctrl+左键多选）"));
    selectInteriorPointButton_ =
        new QPushButton(QStringLiteral("点击高亮内壁生成内部点（多实体必选）"));
    selectInteriorPointButton_->setEnabled(false);
    applyGeometryButton_ =
        new QPushButton(QStringLiteral("确认边界面并应用重力方向"));
    cadLabel_ = new QLabel(QStringLiteral("未加载 CAD，当前使用参数化长方体"));
    cadLabel_->setWordWrap(true);
    resultLabel_ = new QLabel(QStringLiteral("选择一种算法开始计算"));
    resultLabel_->setWordWrap(true);
    resultLabel_->setMinimumWidth(270);

    controls->addLayout(form);
    controls->addWidget(openCadButton_);
    controls->addWidget(selectEntryFaceButton_);
    controls->addWidget(selectInteriorPointButton_);
    controls->addWidget(applyGeometryButton_);
    controls->addWidget(cadLabel_);
    controls->addWidget(latticeButton_);
    controls->addWidget(settlingButton_);
    controls->addWidget(cancelSettlingButton_);
    controls->addWidget(resultLabel_);
    controls->addStretch();

    viewport_ = new OcctViewport;
    root->addLayout(controls);
    root->addWidget(viewport_, 1);
    setCentralWidget(central);

    viewport_->setFaceSelectionCallback(
        [this](const std::vector<TopoDS_Face>& faces) {
            handleFacesSelected(faces);
        });
    viewport_->setPointSelectionCallback(
        [this](const gp_Pnt& seed, const gp_Pnt& entry,
               const TopoDS_Face& face) {
            handlePointSelected(seed, entry, face);
        });
    viewport_->setPointSelectionRejectedCallback(
        [this](const QString& message) {
            cadLabel_->setText(cadDescription_ + QStringLiteral("\n") + message);
        });

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
    connect(cancelSettlingButton_, &QPushButton::clicked, this,
            [this] { cancelSettling(); });
    connect(&settlingWatcher_, &QFutureWatcher<magazine::packing::PackingResult>::finished,
            this, [this] { finishSettling(); });
    connect(&assemblyWatcher_,
            &QFutureWatcher<magazine::cad::AssemblyPackingRegion>::finished,
            this, [this] { finishAssemblyExtraction(); });
    connect(openCadButton_, &QPushButton::clicked, this,
            [this] { openCad(); });
    connect(selectEntryFaceButton_, &QPushButton::clicked, this,
            [this] { beginFaceSelection(); });
    connect(selectInteriorPointButton_, &QPushButton::clicked, this,
            [this] { beginPointSelection(); });
    connect(applyGeometryButton_, &QPushButton::clicked, this,
            [this] { applyGeometrySettings(); });

    if (autoCompute) {
        QTimer::singleShot(0, this, [this] { runLattice(); });
    }
}

MainWindow::~MainWindow() {
    if (settlingCancel_) settlingCancel_->store(true);
    settlingWatcher_.waitForFinished();
    assemblyWatcher_.waitForFinished();
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
        if (assemblyRegion_.has_value()) {
            // The assembly region is already cropped to the reachable
            // component, so the full two-phase reference search is affordable
            // and avoids a systematic phase-dependent undercount.
            latticeOptions.phaseDivisions = 2;
        } else if (cadRegion_.has_value()) {
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
             : QStringLiteral("%1（%2）")
                  .arg(QString::fromStdString(result.method))
                  .arg(assemblyRegion_.has_value()
                           ? QStringLiteral("整车装配体入口连通空间")
                           : QStringLiteral("CAD 真实内腔，已按用户重力方向对齐"))
                  ;
        showResult(method, result.centers.size(),
                   QString::fromStdString(validation.message));
    } catch (const std::exception& error) {
        resultLabel_->setText(QStringLiteral("计算失败：%1")
                                  .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::runSettling() {
    if (settlingWatcher_.isRunning()) return;
    try {
        const magazine::packing::AxisAlignedBox inputBox{
            width_->value(), depth_->value(), height_->value()};
        const double radius = caliber_->currentData().toDouble() / 2.0;
        const auto fallback = magazine::packing::boxPackingRegion(inputBox);
        const auto& region = cadRegion_.has_value() ? *cadRegion_ : fallback;
        magazine::packing::SettlingOptions options;
        if (cadRegion_.has_value()) {
            // CAD clearance is much more expensive than box clearance.  The
            // lower trial budget reaches the same jammed region in a fraction
            // of the time while the final exact validation remains unchanged.
            options.failedInsertionsBeforeStop = 72;
            options.candidateTrialsPerSphere = 12;
            options.relaxationDirections = 10;
            options.maximumRelaxationIterations = 18;
        }
        settlingRegion_ = region;
        settlingCancel_ = std::make_shared<std::atomic_bool>(false);
        const auto cancel = settlingCancel_;
        const auto regionCopy = region;
        latticeButton_->setEnabled(false);
        settlingButton_->setEnabled(false);
        openCadButton_->setEnabled(false);
        selectEntryFaceButton_->setEnabled(false);
        selectInteriorPointButton_->setEnabled(false);
        applyGeometryButton_->setEnabled(false);
        cancelSettlingButton_->setEnabled(true);
        resultLabel_->setText(QStringLiteral("准静态沉降计算中，可点击“停止准静态计算”"));
        settlingWatcher_.setFuture(QtConcurrent::run(
            [regionCopy, radius, options, cancel]() mutable {
                options.cancellationRequested = [cancel] {
                    return cancel->load();
                };
                return magazine::packing::settleWithoutFriction(
                    regionCopy, radius, options);
            }));
    } catch (const std::exception& error) {
        settlingRegion_.reset();
        settlingCancel_.reset();
        resultLabel_->setText(QStringLiteral("计算失败：%1")
                                  .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::cancelSettling() {
    if (!settlingCancel_) return;
    settlingCancel_->store(true);
    cancelSettlingButton_->setEnabled(false);
    resultLabel_->setText(QStringLiteral("正在停止准静态沉降，请稍候…"));
}

void MainWindow::finishSettling() {
    const bool cancelled = settlingCancel_ && settlingCancel_->load();
    try {
        const auto result = settlingWatcher_.result();
        if (cancelled) {
            resultLabel_->setText(QStringLiteral("准静态沉降已停止，未更新显示结果"));
        } else if (settlingRegion_.has_value()) {
            const auto validation = magazine::packing::validatePacking(
                *settlingRegion_, result, 0.15);
            presentPacking(*settlingRegion_, result);
            const QString method = !cadRegion_.has_value()
                ? QStringLiteral("无摩擦准静态沉降")
                : QStringLiteral("无摩擦准静态沉降（%1）")
                      .arg(assemblyRegion_.has_value()
                               ? QStringLiteral("整车装配体入口连通空间")
                               : QStringLiteral("CAD 真实内腔，已按用户重力方向对齐"));
            showResult(method, result.centers.size(),
                       QString::fromStdString(validation.message));
        }
    } catch (const std::exception& error) {
        resultLabel_->setText(QStringLiteral("计算失败：%1")
                                  .arg(QString::fromUtf8(error.what())));
    }
    settlingRegion_.reset();
    settlingCancel_.reset();
    latticeButton_->setEnabled(true);
    settlingButton_->setEnabled(true);
    openCadButton_->setEnabled(true);
    selectEntryFaceButton_->setEnabled(true);
    selectInteriorPointButton_->setEnabled(hasBoundaryFaces_);
    applyGeometryButton_->setEnabled(true);
    cancelSettlingButton_->setEnabled(false);
}

void MainWindow::finishAssemblyExtraction() {
    try {
        const auto extracted = assemblyWatcher_.result();
        assemblyRegion_ = extracted;
        cadRegion_ = extracted.region;
        cadShape_ = extracted.displayShape;
        width_->setValue(extracted.region.bounds.widthMm);
        depth_->setValue(extracted.region.bounds.depthMm);
        height_->setValue(extracted.region.bounds.heightMm);
        viewport_->setFaceSelectionEnabled(false);
        viewport_->setPointSelectionEnabled(false);
        viewport_->clearFaceSelection();
        viewport_->clearSelectionMarker();
        viewport_->setCadShape(cadShape_);
        const gp_Dir gravity = pendingGravity_.value_or(gp_Dir(0.0, 0.0, -1.0));
        cadLabel_->setText(
            QStringLiteral("整车装配体空间提取完成\n源实体：%1，局部障碍：%2，入口连通单元：%3\n选中边界面：%4，生效边界墙：%5，保留边界实体：%6\n边界窗口：%7\n网格尺寸：%8 mm，已按边界面裁剪局部空间\n重力方向：(%9, %10, %11)\n腔体定位：%12\n请点击 FCC/HCP 或准静态沉降开始计算")
                .arg(static_cast<qulonglong>(extracted.sourceSolidCount))
                .arg(static_cast<qulonglong>(extracted.obstacleCount))
                .arg(static_cast<qulonglong>(extracted.reachableCellCount))
                .arg(static_cast<qulonglong>(extracted.selectedBoundaryFaceCount))
                .arg(static_cast<qulonglong>(extracted.boundaryConstraintCount))
                .arg(static_cast<qulonglong>(extracted.boundarySolidCount))
                .arg(extracted.usedTangentialEnvelopeFallback
                         ? QStringLiteral("选面包络回退（仍保留单侧平面和实体碰撞，请核验范围）")
                         : QStringLiteral("选面切向交集"))
                .arg(extracted.cellSizeMm, 0, 'f', 1)
                .arg(gravity.X(), 0, 'f', 3)
                .arg(gravity.Y(), 0, 'f', 3)
                .arg(gravity.Z(), 0, 'f', 3)
                .arg(hasEntryPoint_
                         ? QStringLiteral("使用用户指定内部点")
                         : QStringLiteral("未指定内部点（仅按边界面推断）")));
    } catch (const std::exception& error) {
        cadRegion_.reset();
        assemblyRegion_.reset();
        viewport_->clearPacking();
        viewport_->setCadShape(importedCad_.shape);
        viewport_->setFaceSelectionEnabled(true);
        viewport_->setPointSelectionEnabled(false);
        cadLabel_->setText(explainAssemblyExtractionFailure(
            QString::fromUtf8(error.what())));
    }
    pendingGravity_.reset();
    latticeButton_->setEnabled(true);
    settlingButton_->setEnabled(true);
    openCadButton_->setEnabled(true);
    selectEntryFaceButton_->setEnabled(true);
    selectInteriorPointButton_->setEnabled(false);
    applyGeometryButton_->setEnabled(true);
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
        boundaryFaces_.clear();
        hasBoundaryFaces_ = false;
        hasEntryPoint_ = false;
        cadRegion_.reset();
        assemblyRegion_.reset();
        cadShape_ = imported.shape;
        cadDescription_ = QStringLiteral("%1\n源模型包围盒：%2 x %3 x %4 mm\n实体 %5，壳 %6，面 %7\n%8\n请点击“选择弹仓边界面”，再用 Ctrl+左键选择围成目标空腔的多个内壁面")
            .arg(QString::fromStdString(imported.path))
            .arg(imported.widthMm(), 0, 'f', 2)
            .arg(imported.depthMm(), 0, 'f', 2)
            .arg(imported.heightMm(), 0, 'f', 2)
            .arg(static_cast<qulonglong>(imported.solidCount))
            .arg(static_cast<qulonglong>(imported.shellCount))
            .arg(static_cast<qulonglong>(imported.faceCount))
            .arg(imported.solidCount > 1
                    ? QStringLiteral("检测到装配体，将用所选边界面限定目标自由空间")
                    : QStringLiteral("检测到单体模型，将按所选入口面处理"));
        cadLabel_->setText(cadDescription_);
        viewport_->clearPacking();
        viewport_->setCadShape(cadShape_);
        viewport_->clearSelectionMarker();
        viewport_->clearFaceSelection();
        viewport_->setPointSelectionEnabled(false);
        viewport_->setFaceSelectionEnabled(true);
        selectInteriorPointButton_->setEnabled(false);
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
    assemblyRegion_.reset();
    hasEntryFace_ = false;
    entryFace_.Nullify();
    boundaryFaces_.clear();
    hasBoundaryFaces_ = false;
    hasEntryPoint_ = false;
    cadShape_ = importedCad_.shape;
    viewport_->clearPacking();
    viewport_->setCadShape(cadShape_);
    viewport_->clearSelectionMarker();
    viewport_->clearFaceSelection();
    viewport_->setPointSelectionEnabled(false);
    viewport_->setFaceSelectionEnabled(true);
    selectInteriorPointButton_->setEnabled(false);
    cadLabel_->setText(
        cadDescription_ +
        QStringLiteral("\n正在选择边界面：普通左键单选，Ctrl+左键追加/取消；黄色高亮面将作为弹仓边界。选好后点击“确认边界面并应用重力方向”"));
}

void MainWindow::handleFacesSelected(const std::vector<TopoDS_Face>& faces) {
    boundaryFaces_ = faces;
    hasBoundaryFaces_ = !boundaryFaces_.empty();
    entryFace_.Nullify();
    hasEntryFace_ = false;
    if (!boundaryFaces_.empty()) {
        entryFace_ = boundaryFaces_.front();
        hasEntryFace_ = true;
    }
    if (!hasBoundaryFaces_) {
        cadLabel_->setText(cadDescription_ +
                           QStringLiteral("\n尚未选择边界面；普通左键单选，Ctrl+左键追加/取消"));
        return;
    }
    cadLabel_->setText(
        cadDescription_ +
        QStringLiteral("\n已选择 %1 个边界面（黄色高亮），请确认这些面确实围成目标弹仓后点击“确认边界面并应用重力方向”")
            .arg(static_cast<qulonglong>(boundaryFaces_.size())));
    selectInteriorPointButton_->setEnabled(true);
}

void MainWindow::beginPointSelection() {
    if (!hasImportedCad_) {
        cadLabel_->setText(QStringLiteral("请先打开 STEP / STL 模型"));
        return;
    }
    if (!hasBoundaryFaces_) {
        cadLabel_->setText(QStringLiteral(
            "请先选择至少一个弹仓内侧边界面，再点击高亮内壁生成内部点"));
        return;
    }
    viewport_->setFaceSelectionEnabled(false);
    viewport_->setPointSelectionEnabled(true);
    cadLabel_->setText(
        cadDescription_ +
        QStringLiteral("\n正在生成弹仓内部点：请单击黄色高亮的目标内壁面。程序会从该真实面向自由侧生成红点，鼠标位置不再承担三维深度；不要点击外壁或透明空白。选好后点击“确认边界面并应用重力方向”"));
}

void MainWindow::handlePointSelected(const gp_Pnt& seed, const gp_Pnt& entry,
                                     const TopoDS_Face& face) {
    entrySeedPoint_ = seed;
    entryGatePoint_ = entry;
    hasEntryPoint_ = true;
    entryFace_ = face;
    hasEntryFace_ = !face.IsNull();
    cadLabel_->setText(
        cadDescription_ +
        QStringLiteral("\n已选中弹仓内部点 (%1, %2, %3) mm，入口参考 (%4, %5, %6) mm\n保留的边界面：%7 个\n请点击“确认边界面并应用重力方向”")
            .arg(seed.X(), 0, 'f', 1)
            .arg(seed.Y(), 0, 'f', 1)
            .arg(seed.Z(), 0, 'f', 1)
            .arg(entry.X(), 0, 'f', 1)
            .arg(entry.Y(), 0, 'f', 1)
            .arg(entry.Z(), 0, 'f', 1)
            .arg(static_cast<qulonglong>(boundaryFaces_.size())));
}

void MainWindow::applyGeometrySettings() {
    if (!hasImportedCad_) {
        cadLabel_->setText(QStringLiteral("请先打开 STEP / STL 模型"));
        return;
    }
    if (!hasBoundaryFaces_) {
        cadLabel_->setText(QStringLiteral(
            "请先点击“选择弹仓边界面”，再用 Ctrl+左键选择至少一个围成目标空腔的面"));
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
        if (importedCad_.solidCount > 1) {
            if (!hasEntryPoint_) {
                cadLabel_->setText(QStringLiteral(
                    "请先点击“点击高亮内壁生成内部点（多实体必选）”，再在目标弹仓黄色内壁上单击；"
                    "多实体装配体必须用该点消除多个连通腔体的歧义"));
                return;
            }
            magazine::cad::AssemblyExtractionOptions options;
            options.cellSizeMm = 8.0;
            if (hasEntryPoint_) {
                // Boundary faces constrain the local window; the clicked free
                // point selects which connected cavity inside that window is
                // the magazine. The ray's near boundary is used only as the
                // virtual entry reference for the gravity-aligned gate.
                options.seedPointSource = entrySeedPoint_;
                options.entryPointSource = entryGatePoint_;
            }
            pendingGravity_ = gravity;
            const auto model = importedCad_;
            const auto boundaryFaces = boundaryFaces_;
            // Do not leave an earlier successful region active while a new
            // selection is being evaluated.  A failed extraction must never
            // allow capacity calculation against stale geometry.
            cadRegion_.reset();
            assemblyRegion_.reset();
            viewport_->clearPacking();
            viewport_->setCadShape(importedCad_.shape);
            latticeButton_->setEnabled(false);
            settlingButton_->setEnabled(false);
            openCadButton_->setEnabled(false);
            selectEntryFaceButton_->setEnabled(false);
            selectInteriorPointButton_->setEnabled(false);
            applyGeometryButton_->setEnabled(false);
            cadLabel_->setText(QStringLiteral(
                "正在根据所选边界面提取弹仓自由空间，整车装配体可能需要几十秒，请等待完成"));
            assemblyWatcher_.setFuture(QtConcurrent::run(
                [model, boundaryFaces, gravity, options]() mutable {
                    return magazine::cad::makeAssemblyPackingRegion(
                        model, gravity, boundaryFaces, options);
                }));
            return;
        }
        const auto aligned = magazine::cad::makeGravityAlignedPackingRegion(
            importedCad_, gravity, boundaryFaces_.front());
        cadRegion_ = aligned.region;
        assemblyRegion_.reset();
        cadShape_ = aligned.displayShape;
        width_->setValue(aligned.region.bounds.widthMm);
        depth_->setValue(aligned.region.bounds.depthMm);
        height_->setValue(aligned.region.bounds.heightMm);
        viewport_->setFaceSelectionEnabled(false);
        viewport_->setPointSelectionEnabled(false);
        viewport_->clearFaceSelection();
        viewport_->clearSelectionMarker();
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
