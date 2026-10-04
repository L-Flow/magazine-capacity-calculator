#include "OcctViewport.hpp"

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_SelectionScheme.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Triangulation.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRep_Tool.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Bnd_Box.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Precision.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Quantity_Color.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Standard_Failure.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <V3d_TypeOfVisualization.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_State.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <QMouseEvent>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QString>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace {

Quantity_Color color(int red, int green, int blue) {
    return Quantity_Color(red / 255.0, green / 255.0, blue / 255.0,
                          Quantity_TOC_RGB);
}

struct RayBoundary {
    double parameter{0.0};
    TopoDS_Face face;
    gp_Pnt point;
    double uParameter{0.0};
    double vParameter{0.0};
};

struct InteriorSelection {
    gp_Pnt seed;
    gp_Pnt entry;
    TopoDS_Face entryFace;
};

std::optional<gp_Dir> faceNormalAt(const TopoDS_Face& face,
                                    double uParameter, double vParameter) {
    if (face.IsNull()) return std::nullopt;
    try {
        const Handle(Geom_Surface) geometry = BRep_Tool::Surface(face);
        if (geometry.IsNull()) return std::nullopt;
        GeomLProp_SLProps properties(geometry, uParameter, vParameter, 1,
                                     1.0e-9);
        if (!properties.IsNormalDefined()) return std::nullopt;
        gp_Dir normal = properties.Normal();
        if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
        return normal;
    } catch (const Standard_Failure&) {
        return std::nullopt;
    }
}

std::optional<std::pair<double, double>> lineBoxInterval(
    const Bnd_Box& box, const gp_Pnt& origin, const gp_Dir& direction) {
    if (box.IsVoid()) return std::nullopt;
    double minX = 0.0;
    double minY = 0.0;
    double minZ = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    double maxZ = 0.0;
    box.Get(minX, minY, minZ, maxX, maxY, maxZ);
    const double lower[3] = {minX, minY, minZ};
    const double upper[3] = {maxX, maxY, maxZ};
    const double point[3] = {origin.X(), origin.Y(), origin.Z()};
    const double vector[3] = {direction.X(), direction.Y(), direction.Z()};
    double nearParameter = -std::numeric_limits<double>::infinity();
    double farParameter = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(vector[axis]) < 1.0e-12) {
            if (point[axis] < lower[axis] || point[axis] > upper[axis]) {
                return std::nullopt;
            }
            continue;
        }
        double a = (lower[axis] - point[axis]) / vector[axis];
        double b = (upper[axis] - point[axis]) / vector[axis];
        if (a > b) std::swap(a, b);
        nearParameter = std::max(nearParameter, a);
        farParameter = std::min(farParameter, b);
    }
    if (!(nearParameter < farParameter)) return std::nullopt;
    return std::make_pair(nearParameter, farParameter);
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
        context->SetPixelTolerance(4);
        view = viewer->CreateView();
        Handle(WNT_Window) window =
            new WNT_Window(reinterpret_cast<Aspect_Handle>(owner->winId()));
        view->SetWindow(window);
        if (!window->IsMapped()) window->Map();
        view->SetBackgroundColor(color(29, 36, 46));
        // Do not create the OCCT text-based triedron here.  Its default
        // Courier font is not present on every deployment machine and can
        // abort OpenGL initialization before the model becomes interactive.
        view->ChangeRenderingParams().NbMsaaSamples = 4;
        view->MustBeResized();
        initialized = true;
        rebuild();
    }

    QPoint native(const QPoint& point) const {
        const double ratio = std::max(1.0, owner->devicePixelRatioF());
        return {qRound(point.x() * ratio), qRound(point.y() * ratio)};
    }

    void prepareSolidClassifiers() const {
        if (!solidClassifiers.empty() || cadShape.IsNull()) return;
        for (TopExp_Explorer it(cadShape, TopAbs_SOLID); it.More(); it.Next()) {
            try {
                auto classifier =
                    std::make_shared<BRepClass3d_SolidClassifier>();
                classifier->Load(TopoDS::Solid(it.Current()));
                solidClassifiers.push_back(std::move(classifier));
            } catch (const Standard_Failure&) {
                solidClassifiers.push_back(nullptr);
            }
        }
    }

    bool pointInsideAnySolid(const gp_Pnt& point) const {
        prepareSolidClassifiers();
        for (const auto& classifier : solidClassifiers) {
            if (!classifier) continue;
            try {
                classifier->Perform(point, Precision::Confusion());
                const TopAbs_State state = classifier->State();
                if (state == TopAbs_IN || state == TopAbs_ON) return true;
            } catch (const Standard_Failure&) {
                // A malformed assembly part should not prevent point picking.
            }
        }
        return false;
    }

    std::optional<InteriorSelection> pickInterior(const QPoint& pixel) const {
        if (cadShape.IsNull() || view.IsNull()) return std::nullopt;
        try {
            const QPoint p = native(pixel);
            Standard_Real x = 0.0;
            Standard_Real y = 0.0;
            Standard_Real z = 0.0;
            Standard_Real vx = 0.0;
            Standard_Real vy = 0.0;
            Standard_Real vz = 0.0;
            view->ConvertWithProj(p.x(), p.y(), x, y, z, vx, vy, vz);
            const gp_Pnt origin(x, y, z);
            const gp_Dir direction(vx, vy, vz);

            Bnd_Box bounds;
            BRepBndLib::Add(cadShape, bounds);
            const auto lineRange = lineBoxInterval(bounds, origin, direction);
            if (!lineRange.has_value()) return std::nullopt;

            IntCurvesFace_ShapeIntersector intersector;
            intersector.Load(cadShape, 1.0e-7);
            intersector.Perform(gp_Lin(origin, direction),
                                lineRange->first, lineRange->second);
            if (!intersector.IsDone() || intersector.NbPnt() < 2) {
                return std::nullopt;
            }

            std::vector<RayBoundary> boundaries;
            boundaries.push_back({lineRange->first, TopoDS_Face(), gp_Pnt(),
                                  0.0, 0.0});
            for (Standard_Integer index = 1; index <= intersector.NbPnt();
                 ++index) {
                const double parameter = intersector.WParameter(index);
                if (parameter <= lineRange->first + 1.0e-5 ||
                    parameter >= lineRange->second - 1.0e-5) {
                    continue;
                }
                boundaries.push_back({parameter, intersector.Face(index),
                                      intersector.Pnt(index),
                                      intersector.UParameter(index),
                                      intersector.VParameter(index)});
            }
            boundaries.push_back({lineRange->second, TopoDS_Face(), gp_Pnt(),
                                  0.0, 0.0});
            std::sort(boundaries.begin(), boundaries.end(),
                      [](const RayBoundary& lhs, const RayBoundary& rhs) {
                          return lhs.parameter < rhs.parameter;
                      });

            if (boundaries.size() < 4) return std::nullopt;

            auto isSelectedFace = [&](const TopoDS_Face& face) {
                if (face.IsNull()) return false;
                return std::any_of(
                    selectedFaces.begin(), selectedFaces.end(),
                    [&](const TopoDS_Face& selected) {
                        return !selected.IsNull() && selected.IsSame(face);
                    });
            };

            // A viewport click has no depth.  Use OCCT's visible-face
            // detection as the depth anchor instead of guessing which free
            // interval along the ray the user meant.
            TopoDS_Face detectedFace;
            const QPoint nativePoint = native(pixel);
            context->MoveTo(nativePoint.x(), nativePoint.y(), view,
                            Standard_False);
            if (context->HasDetected()) {
                const Handle(StdSelect_BRepOwner) owner =
                    Handle(StdSelect_BRepOwner)::DownCast(
                        context->DetectedOwner());
                if (!owner.IsNull() && owner->Shape().ShapeType() == TopAbs_FACE) {
                    detectedFace = TopoDS::Face(owner->Shape());
                }
            }

            std::optional<RayBoundary> anchorBoundary;
            if (!detectedFace.IsNull()) {
                for (std::size_t index = 1; index + 1 < boundaries.size();
                     ++index) {
                    if (!boundaries[index].face.IsNull() &&
                        boundaries[index].face.IsSame(detectedFace)) {
                        anchorBoundary = boundaries[index];
                        break;
                    }
                }
            }

            // In multi-solid mode a click on arbitrary empty screen space is
            // deliberately rejected. The user must click one of the yellow
            // selected inner walls, which makes the intended cavity explicit.
            if (!anchorBoundary.has_value() ||
                (!selectedFaces.empty() &&
                 !isSelectedFace(anchorBoundary->face))) {
                return std::nullopt;
            }

            const auto anchorNormal = faceNormalAt(
                anchorBoundary->face, anchorBoundary->uParameter,
                anchorBoundary->vParameter);
            if (anchorNormal.has_value()) {
                struct SideCandidate {
                    bool free{false};
                    gp_Pnt point;
                    double viewScore{-std::numeric_limits<double>::infinity()};
                };
                SideCandidate sides[2];
                const gp_Vec towardViewer = -gp_Vec(direction);
                for (int side = 0; side < 2; ++side) {
                    const double sign = side == 0 ? 1.0 : -1.0;
                    for (const double offset : {2.0, 4.0, 8.0, 12.0,
                                                16.0, 24.0}) {
                        const gp_Pnt candidate = anchorBoundary->point.Translated(
                            gp_Vec(*anchorNormal) * (sign * offset));
                        if (!pointInsideAnySolid(candidate)) {
                            sides[side].free = true;
                            sides[side].point = candidate;
                            sides[side].viewScore =
                                gp_Vec(*anchorNormal).Dot(towardViewer) * sign;
                            break;
                        }
                    }
                }
                int freeSide = -1;
                if (sides[0].free && !sides[1].free) {
                    freeSide = 0;
                } else if (!sides[0].free && sides[1].free) {
                    freeSide = 1;
                } else if (sides[0].free && sides[1].free) {
                    freeSide = sides[0].viewScore >= sides[1].viewScore ? 0 : 1;
                }
                if (freeSide >= 0) {
                    InteriorSelection selection;
                    selection.entry = anchorBoundary->point;
                    selection.seed = sides[freeSide].point;
                    selection.entryFace = anchorBoundary->face;
                    return selection;
                }
                return std::nullopt;
            }

            // Do not fall back to an arbitrary ray interval when a face
            // normal cannot be evaluated; that would reintroduce the
            // external-air ambiguity this interaction is designed to avoid.
            return std::nullopt;

        } catch (const Standard_Failure&) {
            return std::nullopt;
        }
    }

    void refreshSelectionMarker() {
        if (!initialized || context.IsNull()) return;
        if (!pointPresentation.IsNull()) {
            context->Remove(pointPresentation, Standard_False);
            pointPresentation.Nullify();
        }
        if (selectionPoint.has_value()) {
            pointPresentation = new AIS_Shape(
                BRepPrimAPI_MakeSphere(*selectionPoint, 4.0).Shape());
            context->SetColor(pointPresentation, color(255, 80, 80),
                              Standard_False);
            context->Display(pointPresentation, Standard_False);
            context->Deactivate(pointPresentation);
        }
        context->UpdateCurrentViewer();
    }

    static bool sameFace(const TopoDS_Face& lhs, const TopoDS_Face& rhs) {
        return !lhs.IsNull() && !rhs.IsNull() && lhs.IsSame(rhs);
    }

    void refreshFaceSelectionHighlights() {
        if (!initialized || context.IsNull()) return;
        for (const auto& presentation : selectedFacePresentations) {
            if (!presentation.IsNull()) {
                context->Remove(presentation, Standard_False);
            }
        }
        selectedFacePresentations.clear();
        for (const auto& face : selectedFaces) {
            if (face.IsNull()) continue;
            Handle(AIS_Shape) presentation = new AIS_Shape(face);
            context->SetColor(presentation, color(255, 215, 64),
                              Standard_False);
            context->SetTransparency(presentation, 0.18, Standard_False);
            context->SetZLayer(presentation, Graphic3d_ZLayerId_Top);
            context->Display(presentation, Standard_False);
            if (pointSelectionEnabled) {
                context->SetSelectionModeActive(
                    presentation, AIS_Shape::SelectionMode(TopAbs_FACE),
                    Standard_True, AIS_SelectionModesConcurrency_Single,
                    Standard_True);
            } else {
                context->Deactivate(presentation);
            }
            selectedFacePresentations.push_back(presentation);
        }
        context->UpdateCurrentViewer();
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
            if (faceSelectionEnabled || pointSelectionEnabled) {
                context->SetSelectionModeActive(
                    cadPresentation, AIS_Shape::SelectionMode(TopAbs_FACE),
                    Standard_True, AIS_SelectionModesConcurrency_Single,
                    Standard_True);
            } else {
                context->Deactivate(cadPresentation);
            }
        }

        refreshFaceSelectionHighlights();

        if (hasData) {
            const bool lattice = result.method.find("ideal") != std::string::npos;
            spherePresentation = new AIS_Triangulation(makeSphereMesh(result));
            context->SetColor(spherePresentation,
                              lattice ? color(66, 165, 245) : color(255, 183, 77),
                              Standard_False);
            context->Display(spherePresentation, Standard_False);
            context->Deactivate(spherePresentation);
        }
        refreshSelectionMarker();
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
    bool pointSelectionEnabled{false};
    OcctViewport::FaceSelectionCallback faceSelectionCallback;
    OcctViewport::PointSelectionCallback pointSelectionCallback;
    OcctViewport::PointSelectionRejectedCallback
        pointSelectionRejectedCallback;
    std::vector<TopoDS_Face> selectedFaces;
    std::vector<Handle(AIS_Shape)> selectedFacePresentations;
    std::optional<gp_Pnt> selectionPoint;
    mutable std::vector<std::shared_ptr<BRepClass3d_SolidClassifier>>
        solidClassifiers;

    Handle(Aspect_DisplayConnection) displayConnection;
    Handle(OpenGl_GraphicDriver) driver;
    Handle(V3d_Viewer) viewer;
    Handle(V3d_View) view;
    Handle(AIS_InteractiveContext) context;
    Handle(AIS_Shape) boxPresentation;
    Handle(AIS_Shape) cadPresentation;
    Handle(AIS_Shape) pointPresentation;
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
    impl_->solidClassifiers.clear();
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::clearPacking() {
    impl_->hasData = false;
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::setFaceSelectionEnabled(bool enabled) {
    impl_->faceSelectionEnabled = enabled;
    if (enabled) impl_->pointSelectionEnabled = false;
    // Face selection is also used while no packing result is displayed.  The
    // previous hasData guard left the CAD presentation in its old selection
    // mode after opening a model and made left-clicking faces a no-op.
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::setFaceSelectionCallback(FaceSelectionCallback callback) {
    impl_->faceSelectionCallback = std::move(callback);
}

void OcctViewport::clearFaceSelection() {
    impl_->selectedFaces.clear();
    if (impl_->initialized) {
        impl_->refreshFaceSelectionHighlights();
        if (!impl_->view.IsNull()) impl_->view->Redraw();
    }
    if (impl_->faceSelectionCallback) {
        impl_->faceSelectionCallback(impl_->selectedFaces);
    }
}

void OcctViewport::setPointSelectionEnabled(bool enabled) {
    impl_->pointSelectionEnabled = enabled;
    impl_->faceSelectionEnabled = false;
    if (impl_->initialized) impl_->rebuild();
}

void OcctViewport::setPointSelectionCallback(PointSelectionCallback callback) {
    impl_->pointSelectionCallback = std::move(callback);
}

void OcctViewport::setPointSelectionRejectedCallback(
    PointSelectionRejectedCallback callback) {
    impl_->pointSelectionRejectedCallback = std::move(callback);
}

void OcctViewport::clearSelectionMarker() {
    impl_->selectionPoint.reset();
    if (impl_->initialized) impl_->rebuild();
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
    if (event->button() == Qt::LeftButton && impl_->pointSelectionEnabled &&
        !impl_->cadShape.IsNull()) {
        const auto selection = impl_->pickInterior(event->pos());
        if (selection.has_value()) {
            impl_->selectionPoint = selection->seed;
            if (impl_->pointSelectionCallback) {
                impl_->pointSelectionCallback(selection->seed,
                                              selection->entry,
                                              selection->entryFace);
            }
            impl_->refreshSelectionMarker();
            impl_->view->Redraw();
        } else if (impl_->pointSelectionRejectedCallback) {
            impl_->pointSelectionRejectedCallback(QStringLiteral(
                "没有命中黄色高亮的弹仓内壁。请点击黄色面本身，内部点将由该面自动生成；不要点击外部透明区域。"));
        }
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton && impl_->faceSelectionEnabled &&
        !impl_->cadShape.IsNull()) {
        const QPoint p = impl_->native(event->pos());
        impl_->context->MoveTo(p.x(), p.y(), impl_->view, Standard_True);
        TopoDS_Shape selected;
        if (impl_->context->HasDetected()) {
            const Handle(StdSelect_BRepOwner) owner =
                Handle(StdSelect_BRepOwner)::DownCast(
                    impl_->context->DetectedOwner());
            if (!owner.IsNull()) {
                selected = owner->Shape();
            }
            impl_->context->SelectDetected(AIS_SelectionScheme_Replace);
        }
        if (!selected.IsNull() && selected.ShapeType() == TopAbs_FACE) {
            const TopoDS_Face face = TopoDS::Face(selected);
            const bool additive =
                (event->modifiers() & Qt::ControlModifier) != 0;
            auto selectedIt = std::find_if(
                impl_->selectedFaces.begin(), impl_->selectedFaces.end(),
                [&](const TopoDS_Face& candidate) {
                    return Impl::sameFace(candidate, face);
                });
            if (additive) {
                if (selectedIt == impl_->selectedFaces.end()) {
                    impl_->selectedFaces.push_back(face);
                } else {
                    impl_->selectedFaces.erase(selectedIt);
                }
            } else {
                impl_->selectedFaces.clear();
                impl_->selectedFaces.push_back(face);
            }
            impl_->context->ClearSelected(Standard_False);
            impl_->refreshFaceSelectionHighlights();
            if (impl_->faceSelectionCallback) {
                impl_->faceSelectionCallback(impl_->selectedFaces);
            }
        }
        impl_->view->Redraw();
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

