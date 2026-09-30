#include "OcctViewport.hpp"

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Triangulation.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Failure.hxx>
#include <V3d_TypeOfVisualization.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>

#include <QMouseEvent>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QString>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace {

Quantity_Color color(int red, int green, int blue) {
    return Quantity_Color(red / 255.0, green / 255.0, blue / 255.0,
                          Quantity_TOC_RGB);
}

Handle(Poly_Triangulation) makeSphereMesh(
    const magazine::packing::PackingResult& result) {
    constexpr int slices = 12;
    constexpr int stacks = 8;
    constexpr double pi = 3.14159265358979323846;
    constexpr int nodesPerSphere = 2 + (stacks - 1) * slices;
    constexpr int trianglesPerSphere = 2 * slices * (stacks - 1);

    if (result.centers.empty()) return {};

    const auto sphereCount = static_cast<int>(result.centers.size());
    Handle(Poly_Triangulation) mesh = new Poly_Triangulation(
        sphereCount * nodesPerSphere,
        sphereCount * trianglesPerSphere,
        Standard_False,
        Standard_True);

    int triangleIndex = 1;
    for (int sphereIndex = 0; sphereIndex < sphereCount; ++sphereIndex) {
        const auto& center = result.centers[static_cast<std::size_t>(sphereIndex)];
        const int nodeBase = sphereIndex * nodesPerSphere;
        const int top = nodeBase + 1;
        const int bottom = nodeBase + nodesPerSphere;
        const auto ringNode = [nodeBase, slices](int ring, int slice) {
            return nodeBase + 2 + (ring - 1) * slices + slice;
        };

        mesh->SetNode(top, gp_Pnt(center.x, center.y,
                                  center.z + result.sphereRadiusMm));
        mesh->SetNormal(top, gp_Dir(0.0, 0.0, 1.0));

        for (int ring = 1; ring < stacks; ++ring) {
            const double theta = pi * ring / stacks;
            const double sinTheta = std::sin(theta);
            const double cosTheta = std::cos(theta);
            for (int slice = 0; slice < slices; ++slice) {
                const double phi = 2.0 * pi * slice / slices;
                const double nx = sinTheta * std::cos(phi);
                const double ny = sinTheta * std::sin(phi);
                const double nz = cosTheta;
                const int node = ringNode(ring, slice);
                mesh->SetNode(node,
                              gp_Pnt(center.x + result.sphereRadiusMm * nx,
                                     center.y + result.sphereRadiusMm * ny,
                                     center.z + result.sphereRadiusMm * nz));
                mesh->SetNormal(node, gp_Dir(nx, ny, nz));
            }
        }

        mesh->SetNode(bottom, gp_Pnt(center.x, center.y,
                                     center.z - result.sphereRadiusMm));
        mesh->SetNormal(bottom, gp_Dir(0.0, 0.0, -1.0));

        for (int slice = 0; slice < slices; ++slice) {
            const int next = (slice + 1) % slices;
            mesh->SetTriangle(triangleIndex++,
                              Poly_Triangle(top,
                                            ringNode(1, slice),
                                            ringNode(1, next)));
        }
        for (int ring = 1; ring < stacks - 1; ++ring) {
            for (int slice = 0; slice < slices; ++slice) {
                const int next = (slice + 1) % slices;
                const int upper = ringNode(ring, slice);
                const int upperNext = ringNode(ring, next);
                const int lower = ringNode(ring + 1, slice);
                const int lowerNext = ringNode(ring + 1, next);
                mesh->SetTriangle(triangleIndex++,
                                  Poly_Triangle(upper, lower, upperNext));
                mesh->SetTriangle(triangleIndex++,
                                  Poly_Triangle(upperNext, lower, lowerNext));
            }
        }
        for (int slice = 0; slice < slices; ++slice) {
            const int next = (slice + 1) % slices;
            mesh->SetTriangle(triangleIndex++,
                              Poly_Triangle(ringNode(stacks - 1, slice),
                                            bottom,
                                            ringNode(stacks - 1, next)));
        }
    }
    return mesh;
}

} // namespace

struct OcctViewport::Impl {
    explicit Impl(OcctViewport* owner) : owner(owner) {}

    enum class DragMode { None, Rotate, Pan };

    void initialize() {
        if (initialized) return;
        displayConnection = new Aspect_DisplayConnection();
        driver = new OpenGl_GraphicDriver(displayConnection);
        viewer = new V3d_Viewer(driver);
        viewer->SetDefaultLights();
        viewer->SetLightOn();
        context = new AIS_InteractiveContext(viewer);
        view = viewer->CreateView();
        Handle(WNT_Window) window =
            new WNT_Window(reinterpret_cast<Aspect_Handle>(owner->winId()));
        view->SetWindow(window);
        if (!window->IsMapped()) window->Map();
        view->SetBackgroundColor(color(29, 36, 46));
        view->TriedronDisplay(Aspect_TOTP_RIGHT_LOWER, color(235, 239, 244),
                              0.075, V3d_ZBUFFER);
        view->ChangeRenderingParams().NbMsaaSamples = 4;
        view->MustBeResized();
        initialized = true;
        rebuild();
    }

    QPoint native(const QPoint& point) const {
        const double ratio = std::max(1.0, owner->devicePixelRatioF());
        return {qRound(point.x() * ratio), qRound(point.y() * ratio)};
    }

    void rebuild() {
        if (!initialized) return;
        context->RemoveAll(Standard_False);

        if (hasData) {
            const TopoDS_Shape boxShape =
                BRepPrimAPI_MakeBox(box.widthMm, box.depthMm, box.heightMm).Shape();
            boxPresentation = new AIS_Shape(boxShape);
            context->SetColor(boxPresentation, color(165, 177, 190), Standard_False);
            context->SetTransparency(boxPresentation, 0.82, Standard_False);
            context->Display(boxPresentation, Standard_False);
            context->Deactivate(boxPresentation);
        }

        if (!cadShape.IsNull()) {
            cadPresentation = new AIS_Shape(cadShape);
            context->SetColor(cadPresentation, color(130, 210, 150),
                              Standard_False);
            context->SetTransparency(cadPresentation, 0.72, Standard_False);
            context->Display(cadPresentation, Standard_False);
            if (faceSelectionEnabled) {
                context->Activate(cadPresentation,
                                  AIS_Shape::SelectionMode(TopAbs_FACE),
                                  Standard_True);
            } else {
                context->Deactivate(cadPresentation);
            }
        }

        if (hasData) {
            const bool lattice = result.method.find("ideal") != std::string::npos;
            spherePresentation = new AIS_Triangulation(makeSphereMesh(result));
            context->SetColor(spherePresentation,
                              lattice ? color(66, 165, 245) : color(255, 183, 77),
                              Standard_False);
            context->Display(spherePresentation, Standard_False);
            context->Deactivate(spherePresentation);
        }
        context->UpdateCurrentViewer();
        view->FitAll(0.02, Standard_False);
        view->Redraw();
    }

    OcctViewport* owner;
    bool initialized{false};
    bool hasData{false};
    DragMode dragMode{DragMode::None};
    QPoint lastPosition;
    magazine::packing::AxisAlignedBox box;
    magazine::packing::PackingResult result;
    TopoDS_Shape cadShape;
    bool faceSelectionEnabled{false};
    OcctViewport::FaceSelectionCallback faceSelectionCallback;

    Handle(Aspect_DisplayConnection) displayConnection;
    Handle(OpenGl_GraphicDriver) driver;
    Handle(V3d_Viewer) viewer;
    Handle(V3d_View) view;
    Handle(AIS_InteractiveContext) context;
    Handle(AIS_Shape) boxPresentation;
    Handle(AIS_Shape) cadPresentation;
    Handle(AIS_InteractiveObject) spherePresentation;
};

OcctViewport::OcctViewport(QWidget* parent)
    : QWidget(parent), impl_(std::make_unique<Impl>(this)) {
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_NativeWindow);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(640, 480);
}

OcctViewport::~OcctViewport() = default;

void OcctViewport::setPacking(const magazine::packing::AxisAlignedBox& box,
                              const magazine::packing::PackingResult& result) {
    impl_->box = box;
    impl_->result = result;
    impl_->hasData = true;
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::setCadShape(const TopoDS_Shape& shape) {
    impl_->cadShape = shape;
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::clearPacking() {
    impl_->hasData = false;
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::setFaceSelectionEnabled(bool enabled) {
    impl_->faceSelectionEnabled = enabled;
    // Face selection is also used while no packing result is displayed.  The
    // previous hasData guard left the CAD presentation in its old selection
    // mode after opening a model and made left-clicking faces a no-op.
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::setFaceSelectionCallback(FaceSelectionCallback callback) {
    impl_->faceSelectionCallback = std::move(callback);
}

void OcctViewport::fitAll() {
    if (!impl_->initialized || impl_->view.IsNull()) return;
    impl_->view->FitAll(0.02, Standard_True);
}

bool OcctViewport::saveSnapshot(const QString& path) {
    if (path.isEmpty()) {
        return false;
    }
    try {
        // A native OpenGL child may not receive a paint event when the GUI is
        // launched by an automated smoke test, so initialize it explicitly.
        impl_->initialize();
    } catch (const Standard_Failure&) {
        return false;
    }
    if (!impl_->initialized || impl_->view.IsNull()) return false;
    const QByteArray nativePath = path.toLocal8Bit();
    impl_->view->Redraw();
    return impl_->view->Dump(nativePath.constData()) == Standard_True;
}

QPaintEngine* OcctViewport::paintEngine() const {
    return nullptr;
}

void OcctViewport::paintEvent(QPaintEvent*) {
    try {
        impl_->initialize();
        if (!impl_->view.IsNull()) impl_->view->Redraw();
    } catch (const Standard_Failure&) {
        // The main window remains usable and can report algorithm results even
        // if a local OpenGL/OCCT initialization fails.
    }
}

void OcctViewport::resizeEvent(QResizeEvent*) {
    if (impl_->initialized && !impl_->view.IsNull()) impl_->view->MustBeResized();
}

void OcctViewport::mousePressEvent(QMouseEvent* event) {
    if (!impl_->initialized || impl_->view.IsNull()) return;
    if (event->button() == Qt::LeftButton && impl_->faceSelectionEnabled &&
        !impl_->cadShape.IsNull()) {
        const QPoint p = impl_->native(event->pos());
        impl_->context->MoveTo(p.x(), p.y(), impl_->view, Standard_True);
        impl_->context->Select(Standard_True);
        if (impl_->context->HasSelectedShape()) {
            const TopoDS_Shape selected = impl_->context->SelectedShape();
            if (!selected.IsNull() && selected.ShapeType() == TopAbs_FACE &&
                impl_->faceSelectionCallback) {
                impl_->faceSelectionCallback(TopoDS::Face(selected));
            }
        }
        event->accept();
        return;
    }
    impl_->lastPosition = event->pos();
    if (event->button() == Qt::MiddleButton) {
        if ((event->modifiers() & Qt::ControlModifier) != 0) {
            impl_->dragMode = Impl::DragMode::Pan;
        } else {
            impl_->dragMode = Impl::DragMode::Rotate;
            const QPoint p = impl_->native(event->pos());
            impl_->view->StartRotation(p.x(), p.y(), 0.4);
        }
        event->accept();
    }
}

void OcctViewport::mouseMoveEvent(QMouseEvent* event) {
    if (!impl_->initialized || impl_->view.IsNull()) return;
    if (impl_->dragMode == Impl::DragMode::Rotate) {
        const QPoint p = impl_->native(event->pos());
        impl_->view->Rotation(p.x(), p.y());
        impl_->view->Redraw();
    } else if (impl_->dragMode == Impl::DragMode::Pan) {
        const QPoint delta = impl_->native(event->pos() - impl_->lastPosition);
        impl_->view->Pan(delta.x(), -delta.y(), 1.0, Standard_True);
    }
    impl_->lastPosition = event->pos();
}

void OcctViewport::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        impl_->dragMode = Impl::DragMode::None;
        event->accept();
    }
}

void OcctViewport::mouseDoubleClickEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        fitAll();
        event->accept();
    }
}

void OcctViewport::wheelEvent(QWheelEvent* event) {
    if (!impl_->initialized || impl_->view.IsNull()) return;
    const QPoint p = impl_->native(event->position().toPoint());
    impl_->view->StartZoomAtPoint(p.x(), p.y());
    const int movement = event->angleDelta().y() > 0 ? 12 : -12;
    impl_->view->ZoomAtPoint(0, 0, movement, movement);
    impl_->view->Redraw();
    event->accept();
}

