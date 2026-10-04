#pragma once

#include "packing/AxisAlignedBox.hpp"
#include "packing/PackingTypes.hpp"

#include <memory>
#include <functional>
#include <vector>

#include <QWidget>
#include <gp_Pnt.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

class QString;

class QMouseEvent;
class QPaintEngine;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

class OcctViewport final : public QWidget {
public:
    using FaceSelectionCallback =
        std::function<void(const std::vector<TopoDS_Face>&)>;
    using PointSelectionCallback =
        std::function<void(const gp_Pnt&, const gp_Pnt&, const TopoDS_Face&)>;
    using PointSelectionRejectedCallback = std::function<void(const QString&)>;

    explicit OcctViewport(QWidget* parent = nullptr);
    ~OcctViewport() override;

    void setPacking(const magazine::packing::AxisAlignedBox& box,
                    const magazine::packing::PackingResult& result);
    void setCadShape(const TopoDS_Shape& shape);
    void clearPacking();
    void setFaceSelectionEnabled(bool enabled);
    void setFaceSelectionCallback(FaceSelectionCallback callback);
    void clearFaceSelection();
    void setPointSelectionEnabled(bool enabled);
    void setPointSelectionCallback(PointSelectionCallback callback);
    void setPointSelectionRejectedCallback(
        PointSelectionRejectedCallback callback);
    void clearSelectionMarker();
    void fitAll();
    bool saveSnapshot(const QString& path);

protected:
    QPaintEngine* paintEngine() const override;
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

