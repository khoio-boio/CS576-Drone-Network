"""
run_ui.py — UI-only preview for your Drone GUI
----------------------------------------------
Purpose:
- Loads a Qt Designer .ui file (no backend).
- Renders a 2D map with clickable mock drones.
- Shows small battery icons per drone (3/2/1/empty; empty flashes).
- Clicking a drone highlights it and switches the right panel to Drone details.
- Clicking *empty* map switches right panel back to Fleet.

Read this file top-to-bottom. All tunables live in the CONFIG section.
Every “why” is explained, so edits are safe and obvious.

NOTE: This is a *TEMPORARY MOCK* (no real networking/physics). It is UI scaffolding.
"""

# =========================
# Imports
# =========================
import sys
import random
from pathlib import Path
from typing import List, Dict

from PySide6 import QtWidgets, QtCore, QtGui
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile

import pyqtgraph as pg
import numpy as np


# =========================
# CONFIG
# =========================
# World (meters). This defines the coordinate system the sim runs in.
WORLD_WIDTH_M: int  = 1000          # X spans 0..WORLD_WIDTH_M
WORLD_HEIGHT_M: int = 1000          # Y spans 0..WORLD_HEIGHT_M

# View limits (meters). Allow panning/zoom slightly beyond world for UX.
# If you want strict clamp to world, set MARGIN_OUTSIDE_WORLD_M = 0.
MARGIN_OUTSIDE_WORLD_M: int = 500

# Zoom constraints (meters). Prevents accidental infinite zoom.
MIN_ZOOM_RANGE_M: int = 40          # smallest width/height the user can zoom in to
MAX_ZOOM_RANGE_X: int = WORLD_WIDTH_M + MARGIN_OUTSIDE_WORLD_M
MAX_ZOOM_RANGE_Y: int = WORLD_HEIGHT_M + MARGIN_OUTSIDE_WORLD_M

# Map styling
MAP_BACKGROUND_HEX: str = "#f5f6f8"
GRID_ALPHA: float        = 0.25      # 0..1 transparency for grid
AXIS_LABEL_X: str        = "X (m)"
AXIS_LABEL_Y: str        = "Y (m)"

# TODO: Mock data (TEMP) — how many drones to draw, and randomness seed for reproducibility
NUM_MOCK_DRONES: int = 50
RNG_SEED: int        = 42

# Drone dot styling
SCATTER_SIZE_PX: int          = 10
SCATTER_COLOR_RGBA: tuple[int,int,int,int] = (0, 122, 255, 200)  # semi-transparent blue

# Selected drone ring styling
SELECT_RING_SIZE_PX: int  = 16
SELECT_RING_COLOR_HEX: str = "#1f6feb"  # blue ring

# Battery icon rendering
BATTERY_ICON_W_PX: int = 26
BATTERY_ICON_H_PX: int = 12
BATTERY_ICON_OFFSET_PX: int = 8         # icon offset from each dot (top-right)
BATTERY_FLASH_INTERVAL_MS: int = 500    # flash freq for <25%

# Battery thresholds (%). >=75 → 3 bars; >=50 → 2 bars; >=25 → 1 bar; else 0 bars (empty)
BATT_THRESH_3: float = 75.0
BATT_THRESH_2: float = 50.0
BATT_THRESH_1: float = 25.0
BATT_EMPTY_WARN_COLOR_HEX: str = "#e74c3c"  # red for empty case

# .ui file location (this script is inside the /ui folder, so the UI file is local)
UI_PATH: str = "mainwindow.ui"

# Window initial size (pixels). This is the outer window; map area will be smaller due to right panel.
WINDOW_WIDTH_PX: int  = 1200
WINDOW_HEIGHT_PX: int = 800


# =========================
# Derived/global helpers (don’t usually need to edit)
# =========================
RNG = random.Random(RNG_SEED)  # deterministic randomness for repeatable layout


# =========================
# Utility functions — small, focused helpers with clear behavior
# =========================
def battery_bars(pct: float) -> int:
    """
    Convert battery percent to number of bars:
      - >= BATT_THRESH_3 → 3 bars
      - >= BATT_THRESH_2 → 2 bars
      - >= BATT_THRESH_1 → 1 bar
      - else             → 0 bars (empty)
    Why: Visual simplification. You can change thresholds globally above.
    """
    if pct >= BATT_THRESH_3: return 3
    if pct >= BATT_THRESH_2: return 2
    if pct >= BATT_THRESH_1: return 1
    return 0


def make_battery_pixmap(pct: float, flashing_on: bool) -> QtGui.QPixmap:
    """
    Draw a compact battery icon to overlay near each drone.
    - Size controlled by BATTERY_ICON_W_PX / BATTERY_ICON_H_PX.
    - When bars=0 (<25%), draws a red border and flashes the inner area softly.
    Why: Quick glance UI; exact percent also shown in the Drone details panel.
    """
    bars = battery_bars(pct)
    w, h = BATTERY_ICON_W_PX, BATTERY_ICON_H_PX

    pm = QtGui.QPixmap(w, h)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)

    # Outer case (red if empty)
    pen_color = QtGui.QColor("#d0d3d8") if bars > 0 else QtGui.QColor(BATT_EMPTY_WARN_COLOR_HEX)
    pen = QtGui.QPen(pen_color)
    pen.setWidthF(1.2 if bars > 0 else 1.4)
    p.setPen(pen)
    p.setBrush(QtCore.Qt.NoBrush)
    p.drawRoundedRect(1, 1, w - 5, h - 2, 2, 2)  # room for the 'nub' at right

    # Nub (connector)
    p.setPen(QtCore.Qt.NoPen)
    nub_color = QtGui.QColor("#b0b4bb") if bars > 0 else QtGui.QColor(BATT_EMPTY_WARN_COLOR_HEX)
    p.setBrush(nub_color)
    p.drawRect(w - 5, h // 2 - 2, 4, 4)

    # Bars region (3 possible bars)
    gap = 2
    bar_w = int((w - 9 - gap * 2) / 3)
    x0, y0 = 3, 3
    for i in range(3):
        rect = QtCore.QRectF(x0 + i * (bar_w + gap), y0, bar_w, h - 6)
        if i < bars:
            p.setBrush(QtGui.QColor("#2c3e50"))      # filled bar
            p.setPen(QtCore.Qt.NoPen)
            p.drawRect(rect)
        elif bars == 0 and flashing_on:
            # Flash the empty area faint red to draw attention to critically low battery.
            p.setBrush(QtGui.QColor(231, 76, 60, 90))
            p.setPen(QtCore.Qt.NoPen)
            p.drawRect(rect)

    p.end()
    return pm


# =========================
# Main Window class — owns all UI wiring (TEMP: UI-only scaffold)
# =========================
class App(QtWidgets.QMainWindow):
    """
    This class:
      - Loads the .ui layout.
      - Inserts the pyqtgraph map into `mapContainer`.
      - Creates TEMP mock drones and draws them as a ScatterPlotItem.
      - Overlays battery icons per drone.
      - Handles click interactions to swap the right panel content.
      - Handles background clicks to return to the Fleet panel.

    IMPORTANT: No backend logic or real networking; this is purely for UI/UX iteration.
    """

    # ---------- Map background click: return to Fleet ----------
    def _on_scene_clicked(self, ev):
        """
        Called when any mouse click occurs on the scene (background).
        We *ignore* clicks already handled by child items (like drone points).
        Why: If you clicked a drone, the scatter already selected it. If not, show Fleet.
        """
        # If a child item (e.g., drone dot) accepted the click, do nothing here.
        if hasattr(ev, "isAccepted") and ev.isAccepted():
            return

        # Only react to left-clicks for simplicity.
        if ev.button() != QtCore.Qt.LeftButton:
            return

        # Empty-map click → clear selection and show Fleet page (assumes rightStack page 0 is Fleet).
        self.selected_idx = None
        if hasattr(self, "sel_marker"):
            self.sel_marker.setData([])  # remove highlight ring

        right_stack = self.ui.findChild(QtWidgets.QStackedWidget, "rightStack")
        if right_stack:
            right_stack.setCurrentIndex(0)

        # Status back to default
        sb = self.ui.findChild(QtWidgets.QStatusBar)
        if sb is None and hasattr(self, "statusBar"):
            sb = self.statusBar()
        if sb:
            sb.showMessage("Paused • Time: 0.0 s • FPS: -- • Selected: None")

    # ---------- Standard Qt init ----------
    def __init__(self):
        super().__init__()

        # --- 1) Load .ui from Designer ---
        # Why: Separate visual layout (.ui) from Python code. Lets you rearrange UI without touching code.
        f = QFile(str(UI_PATH))
        if not f.exists():
            raise SystemExit(f"Cannot find {UI_PATH}. Create it in Qt Designer first.")
        f.open(QFile.ReadOnly)
        w = QUiLoader().load(f)
        f.close()
        self.setCentralWidget(w)
        self.ui = w

        # Provide world size to instance (handy if helpers want to read it)
        self.WORLD_W = WORLD_WIDTH_M
        self.WORLD_H = WORLD_HEIGHT_M

        # --- 2) Insert a pyqtgraph PlotWidget into 'mapContainer' ---
        # Why: pyqtgraph gives smooth pan/zoom and easy scatter plots, perfect for a live map.
        self.plot = pg.PlotWidget(background=MAP_BACKGROUND_HEX)
        self.plot.setAspectLocked(True, 1)                       # keep 1 m in X equal to 1 m in Y
        self.plot.showGrid(x=True, y=True, alpha=GRID_ALPHA)
        self.plot.setRange(xRange=[0, self.WORLD_W], yRange=[0, self.WORLD_H], padding=0.02)
        vb = self.plot.getViewBox()
        vb.setLimits(
            # Pan bounds (extend beyond world by MARGIN_OUTSIDE_WORLD_M for UX)
            xMin=0 - MARGIN_OUTSIDE_WORLD_M,
            xMax=self.WORLD_W + MARGIN_OUTSIDE_WORLD_M,
            yMin=0 - MARGIN_OUTSIDE_WORLD_M,
            yMax=self.WORLD_H + MARGIN_OUTSIDE_WORLD_M,
            # Zoom bounds (don’t allow “too far in” or “too far out”)
            minXRange=MIN_ZOOM_RANGE_M,
            minYRange=MIN_ZOOM_RANGE_M,
            maxXRange=MAX_ZOOM_RANGE_X,
            maxYRange=MAX_ZOOM_RANGE_Y,
        )
        self.plot.getAxis("bottom").setLabel(AXIS_LABEL_X)
        self.plot.getAxis("left").setLabel(AXIS_LABEL_Y)

        # Put the plot into the placeholder widget from .ui
        map_container = self.ui.findChild(QtWidgets.QWidget, "mapContainer")
        if map_container is None:
            raise SystemExit("UI error: 'mapContainer' widget not found. Name it exactly in Qt Designer.")
        # Use/ensure a zero-margin VBox so the plot fills the area
        lay = map_container.layout() or QtWidgets.QVBoxLayout(map_container)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.plot)

        # Clicking background should revert to Fleet panel
        self.plot.scene().sigMouseClicked.connect(self._on_scene_clicked)

        # --- 3) Build TEMP mock drones (UI-only) ---
        # Why: Visualize and test UI interactions before wiring real data.
        self.drones: List[Dict] = []
        for i in range(NUM_MOCK_DRONES):
            x = RNG.random() * self.WORLD_W
            y = RNG.random() * self.WORLD_H
            speed = RNG.uniform(2.0, 10.0)    # TEMP display only
            batt = RNG.uniform(10, 100)       # TEMP display only
            self.drones.append({
                "id": i + 1,
                "x": x, "y": y,
                "speed": speed,
                "battery": batt,
            })

        # Optional: O(1) lookup by ID if/when you add “Search by ID”
        self.id2idx = {d["id"]: i for i, d in enumerate(self.drones)}

        # --- 4) Add scatter plot for drone dots ---
        # Why: Single item that manages many points; supports click events.
        self.scatter = pg.ScatterPlotItem(
            size=SCATTER_SIZE_PX,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(*SCATTER_COLOR_RGBA),
            name="drones",
        )
        self.plot.addItem(self.scatter)
        self._redraw_points()
        self.scatter.sigClicked.connect(self._on_points_clicked)  # point selection

        # Selected-drone ring (a separate scatter with 1 big hollow point)
        self.sel_marker = pg.ScatterPlotItem(
            size=SELECT_RING_SIZE_PX,
            pen=pg.mkPen(SELECT_RING_COLOR_HEX, width=2),
            brush=pg.mkBrush(0, 0, 0, 0)
        )
        self.plot.addItem(self.sel_marker)
        self.selected_idx = None

        # --- 5) Battery icon overlays (graphics items) ---
        # Why: Annotate each drone with a compact battery indicator.
        self.icon_items: List[QtWidgets.QGraphicsPixmapItem] = []
        for _ in self.drones:
            item = QtWidgets.QGraphicsPixmapItem()
            self.plot.addItem(item)  # pyqtgraph wraps QGraphicsItem into the scene
            self.icon_items.append(item)

        self._flash_on = True
        self._place_icons()  # initial placement

        # Flash timer toggles empty-battery fill to draw attention
        self.flash_timer = QtCore.QTimer(self, interval=BATTERY_FLASH_INTERVAL_MS)
        self.flash_timer.timeout.connect(self._tick_flash)
        self.flash_timer.start()

        # --- 6) Right panel defaults ---
        right_stack = self.ui.findChild(QtWidgets.QStackedWidget, "rightStack")
        if right_stack:
            right_stack.setCurrentIndex(0)  # assume 0 = Fleet, 1 = Drone

        # --- 7) Status bar (MainWindow only) ---
        sb = self.ui.findChild(QtWidgets.QStatusBar)
        if sb is None and hasattr(self, "statusBar"):
            sb = QtWidgets.QStatusBar(self)
            self.setStatusBar(sb)
        if sb is not None:
            sb.showMessage("Paused • Time: 0.0 s • FPS: -- • Selected: None")

        # --- 8) Search wiring (Enter or click OK button triggers search)
        self.search_edit = self.ui.findChild(QtWidgets.QLineEdit, "searchByID")
        self.search_btn  = self.ui.findChild(QtWidgets.QPushButton, "droneSearchOK")

        if self.search_edit:
            self.search_edit.returnPressed.connect(self._on_search)
        if self.search_btn:
            self.search_btn.clicked.connect(self._on_search)

        # Title for clarity in the window manager
        self.setWindowTitle("Drone Fleet — UI Preview (Unlinked)")

    # ---------- Internal: draw/update helpers ----------
    def _redraw_points(self):
        """
        Push drone (x,y) positions into the scatter plot.
        Why: If positions change (e.g., when you add formations/mobility), call this to refresh.
        """
        pts = np.array([[d["x"], d["y"]] for d in self.drones], dtype=float)
        spots = [{"pos": (p[0], p[1]), "data": idx} for idx, p in enumerate(pts)]
        self.scatter.setData(spots)

    def _place_icons(self):
        """
        Position battery icons near each drone dot.
        Why: Runs initially and whenever flash toggles.
        """
        for idx, d in enumerate(self.drones):
            pm = make_battery_pixmap(d["battery"], flashing_on=(self._flash_on and d["battery"] < BATT_THRESH_1))
            self.icon_items[idx].setPixmap(pm)
            # Offset to top-right of the dot (world coordinates; same scale as points)
            self.icon_items[idx].setPos(d["x"] + BATTERY_ICON_OFFSET_PX, d["y"] + BATTERY_ICON_OFFSET_PX)

    def _tick_flash(self):
        """
        Timer toggles a flag every BATTERY_FLASH_INTERVAL_MS to animate low-battery icons.
        """
        self._flash_on = not self._flash_on
        self._place_icons()

    # ---------- Interaction ----------
    def _on_points_clicked(self, plot, points):
        """
        Called when a drone dot is clicked.
        points: list of pyqtgraph points; we use the first one.
        """
        if not points:
            return
        idx = points[0].data()  # we stored 'data' = index at _redraw_points
        self._select_drone(idx)

    def _select_drone(self, idx: int):
        """
        Mark a drone as selected, draw the highlight ring, and fill the right panel (Drone page).
        Note: We *do not* recenter/zoom here to preserve your manual pan/zoom behavior.
              If later you want a “center on selected” button, call plot.setRange() there.
        """
        self.selected_idx = idx
        d = self.drones[idx]

        # 1) Highlight ring
        self.sel_marker.setData([{"pos": (d["x"], d["y"])}])

        # 2) Switch right panel to Drone page (assumes page index 1 is Drone)
        right_stack = self.ui.findChild(QtWidgets.QStackedWidget, "rightStack")
        if right_stack:
            right_stack.setCurrentIndex(1)

        # 3) Fill individual Drone fields (labels must exist in your .ui)
        def set_text(name: str, text: str):
            lab = self.ui.findChild(QtWidgets.QLabel, name)
            if lab:
                lab.setText(text)

        set_text("valId", str(d["id"]))
        set_text("valRole", "Follower")     # TEMP placeholder until backend defines roles
        set_text("valParent", "--")         # TEMP placeholder
        set_text("valChildren", "--")       # TEMP placeholder
        set_text("valNeighbors", "--")      # TEMP placeholder
        set_text("valPos", f"({d['x']:.1f}, {d['y']:.1f})")
        set_text("valSpeed", f"{d['speed']:.1f} m/s")
        set_text("valBatt", f"{d['battery']:.0f}%")
        set_text("valState", "idle")        # TEMP placeholder

        # 4) Status message
        if hasattr(self, "statusBar") and self.statusBar():
            self.statusBar().showMessage(f"Paused • Time: 0.0 s • FPS: -- • Selected: #{d['id']}")

    def _on_search(self):
        """
        Read drone ID from searchByID, select that drone, switch to the Drone page,
        and center with a small zoom so it's visible even if off-screen.
        """
        if not hasattr(self, "search_edit") or self.search_edit is None:
            return

        raw = self.search_edit.text().strip()

        # Invalid input (non numeric)
        if not raw.isdigit():
            self._show_status("Search: enter a numeric drone ID (e.g., 12)")
            self._flash_edit_border(self.search_edit, bad=True)
            self.search_edit.clear() # clear search bar
            return

        did = int(raw)
        idx = self.id2idx.get(did)

        # drone not found
        if idx is None:
            self._show_status(f"Drone #{did} not found")
            self._flash_edit_border(self.search_edit, bad=True)
            self.search_edit.clear() 
            return

        # Found: select and center with a small zoom window
        self._flash_edit_border(self.search_edit, bad=False)
        self._select_drone(idx)                # updates right panel & highlight
        self._center_on_drone(idx, span=120.0) # ~120×120 m view around the drone
        self._show_status(f"Selected #{did}")
        self.search_edit.clear()

    def _center_on_drone(self, idx: int, span: float = 120.0):
        """
        Center and slightly zoom around the given drone.
        'span' is the width/height of the view in meters (kept square).
        Clamped to world + allowed margins.
        """
        d = self.drones[idx]
        x0, y0 = d["x"], d["y"]
        half = span * 0.5

        # Compute desired ranges
        x1, x2 = x0 - half, x0 + half
        y1, y2 = y0 - half, y0 + half

        # Clamp to world + margins
        xmin = 0 - MARGIN_OUTSIDE_WORLD_M
        ymin = 0 - MARGIN_OUTSIDE_WORLD_M
        xmax = self.WORLD_W + MARGIN_OUTSIDE_WORLD_M
        ymax = self.WORLD_H + MARGIN_OUTSIDE_WORLD_M

        x1 = max(xmin, x1); y1 = max(ymin, y1)
        x2 = min(xmax, x2); y2 = min(ymax, y2)

        self.plot.setRange(xRange=[x1, x2], yRange=[y1, y2], padding=0.02)

    def _show_status(self, msg: str, ms: int = 3000):
        """
        Show a brief message in the status bar (falls back gracefully).
        """
        sb = self.ui.findChild(QtWidgets.QStatusBar)
        if sb is None and hasattr(self, "statusBar"):
            sb = self.statusBar()
        if sb:
            sb.showMessage(msg, ms)

    def _flash_edit_border(self, edit: QtWidgets.QLineEdit, bad: bool):
        """
        Soft visual feedback on the search box:
        - bad=True → light red background + red border
        - bad=False → reset to normal
        """
        if bad:
            edit.setStyleSheet("QLineEdit { background: #ffe6e6; border: 1px solid #e74c3c; }")
        else:
            edit.setStyleSheet("")


# =========================
# Bootstrapping
# =========================
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = App()
    win.resize(WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
