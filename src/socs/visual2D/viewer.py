"""A small PySide6 viewer for live SOCS visual states."""

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
)


class AGVView(QGraphicsView):
    """Show a route and AGV, with interactive and automatic run controls."""

    def __init__(
        self,
        state_buffer,
        route,
        entity_id,
        on_advance,
        parent=None,
        on_auto=None,
        on_interactive=None,
    ):
        super().__init__(parent)
        self.state_buffer = state_buffer
        self.entity_id = entity_id
        self.on_advance = on_advance
        self.on_auto = on_auto
        self.on_interactive = on_interactive
        self._revision = -1
        self._pan_origin = None

        self.setWindowTitle("SOCS PySide6 AGV Demo")
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setBackgroundBrush(QColor("#202833"))
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse
        )

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self._draw_layout(route)
        self._draw_agv()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._sync_state)
        self._timer.start(30)
        self._sync_state()

    @staticmethod
    def _to_scene(point):
        """Map world XY coordinates to screen coordinates (Y points down)."""
        return QPointF(point[0] * 80.0, -point[1] * 80.0)

    def _draw_layout(self, route):
        points = [self._to_scene(point) for point in route]
        points.append(points[0])

        path = QPainterPath(points[0])
        for point in points[1:]:
            path.lineTo(point)
        route_item = self.scene.addPath(
            path,
            QPen(QColor("#778493"), 5.0, Qt.PenStyle.DashLine),
        )
        route_item.setZValue(0)

        for index, point in enumerate(points[:-1]):
            marker = self.scene.addEllipse(
                QRectF(point.x() - 7, point.y() - 7, 14, 14),
                QPen(QColor("#d6dee8"), 1.5),
                QBrush(QColor("#465362")),
            )
            marker.setZValue(1)
            label = self.scene.addSimpleText(f"P{index + 1}")
            label.setBrush(QBrush(QColor("#d6dee8")))
            label.setPos(point.x() + 10, point.y() + 8)
            label.setZValue(1)

        self.scene.setSceneRect(
            self.scene.itemsBoundingRect().adjusted(-90, -90, 90, 90)
        )
        self.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def _draw_agv(self):
        arrow = QPolygonF(
            [
                QPointF(24, 0),
                QPointF(-15, -12),
                QPointF(-9, 0),
                QPointF(-15, 12),
            ]
        )
        self.agv_item = QGraphicsPolygonItem(arrow)
        self.agv_item.setBrush(QBrush(QColor("#28a9ff")))
        self.agv_item.setPen(QPen(QColor("#e8f5ff"), 2.0))
        self.agv_item.setZValue(3)
        self.scene.addItem(self.agv_item)

        self.status_item = QGraphicsSimpleTextItem("Simulation time: 0.0")
        self.status_item.setBrush(QBrush(QColor("#f0f3f6")))
        self.status_item.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations,
            True,
        )
        self.status_item.setPos(
            self.scene.sceneRect().left() + 12,
            self.scene.sceneRect().top() + 12,
        )
        self.status_item.setZValue(4)
        self.scene.addItem(self.status_item)

    def _sync_state(self):
        revision, simulation_time, entities = self.state_buffer.read()
        if revision == self._revision:
            return

        entity = entities.get(self.entity_id)
        if entity is not None:
            self.agv_item.setPos(self._to_scene(entity.position))
            self.agv_item.setRotation(-entity.heading)
            self.agv_item.setVisible(entity.visible)
            if entity.color is not None:
                red, green, blue, alpha = entity.color
                self.agv_item.setBrush(
                    QBrush(QColor.fromRgbF(red, green, blue, alpha))
                )

        self.status_item.setText(f"Simulation time: {simulation_time:.1f}")
        self._revision = revision

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Space:
            self.on_advance()
            event.accept()
        elif event.key() == Qt.Key.Key_A and self.on_auto is not None:
            self.on_auto()
            event.accept()
        elif event.key() == Qt.Key.Key_I and self.on_interactive is not None:
            self.on_interactive()
            event.accept()
        elif event.key() == Qt.Key.Key_F:
            self.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
            event.accept()
        else:
            super().keyPressEvent(event)

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 1.15 if delta > 0 else 1.0 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.RightButton:
            self._pan_origin = event.position().toPoint()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan_origin is not None:
            current = event.position().toPoint()
            delta = current - self._pan_origin
            self._pan_origin = current
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.RightButton and self._pan_origin is not None:
            self._pan_origin = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)
