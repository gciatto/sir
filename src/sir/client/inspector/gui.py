import gi
from morse.builder.bpymorse import properties

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib
from gi.repository import cairo

from collections import namedtuple
from gciatto.gi.mouse import HighLevelMouseObservable
from cmath import phase
from res import paths
from threading import Thread
from gciatto.gi.drawing import *
from math import radians
import time


class MeanAdapter:

    def _get_pose(self) -> complex:
        return complex(0, 0)

    def _get_bearing(self) -> float:
        return 0

    def _get_covariances(self):
        return ((1, 0), (0, 1))

    @property
    def pose(self) -> complex:
        return self._get_pose()

    @property
    def bearing(self) -> float:
        return self._get_bearing()

    @property
    def covariances(self):
        return self._get_covariances()


class InspectorModel:
    def __init__(self, initial_pose=complex(0, 0), initial_bearing=.0):
        self.origin = initial_pose
        self.orientation = initial_bearing
        self.pose = initial_pose
        self.bearing = initial_bearing
        self.state = MeanAdapter()
        self.landmarks = ()


class InspectorController:

    def __init__(self, canvas, model: InspectorModel):
        super().__init__()
        self._model = model
        self._canvas = canvas
        self._size = complex(0, 0)
        self._center = complex(0, 0)
        self._scroll = 0
        self._rotation = 0
        self._canvas_mouse_listener = HighLevelMouseObservable(
            drag=self.on_canvas_drag_notify_event,
            released=self.on_canvas_button_clicked
        )

    def redraw(self):
        self._canvas.queue_draw()
        return True

    def _get_mouse_pressed(self):
        return next((k for k, v in self.mouse_pressed.items() if v), None)

    def on_main_window_delete_event(self, *args):
        Gtk.main_quit(*args)

    def on_canvas_draw(self, canvas, cc: cairo.Context):
        self._draw_world(canvas, cc)
        self._draw_frame(canvas, cc)
        return True

    def _draw_world(self, canvas, cc: cairo.Context):
        cc.translate(self._center.real, self._center.imag)
        zoom_factor = zoom(self._scroll)
        cc.scale(zoom_factor, -zoom_factor)
        cc.rotate(-self._rotation)

        draw_axes(canvas, cc)

        self._draw_actual_robot(canvas, cc)

    def _draw_frame(self, canvas, cc):
        cc.save()
        cc.set_source_rgb(0, 0, 1)

        cc.translate(self._model.origin.real, self._model.origin.imag)
        cc.rotate(self._model.orientation)

        draw_axes(canvas, cc)
        cc.restore()

    def _draw_actual_robot(self, canvas, cc):
        cc.save()
        cc.set_source_rgb(0, 1, 0)
        cc.translate(self._model.pose.real, self._model.pose.imag)
        cc.rotate(self._model.bearing)

        draw_active(canvas, cc)
        cc.restore()

    def _draw_expected_obstacles(self, canvas, cc):
        cc.save()
        cc.set_source_rgb(1, 0, 0)
        for lmk in self._model.landmarks:
            cc.save()
            cc.translate(lmk.pose.real, lmk.pose.imag)
            draw_passive(canvas, cc)
            cc.restore()
        cc.restore()

    def on_canvas_size_allocate(self, canvas, rect):
        self._size = complex(rect.width, rect.height)
        self._center = self._size / 2

    def on_canvas_motion_notify_event(self, canvas, event):
        return self._canvas_mouse_listener.on_mouse_movement(canvas, event)

    def on_canvas_drag_notify_event(self, canvas, button, curr, prev, initial):
        center = self._center
        if button == 1:
            self._center += curr - prev
            self.redraw()
            return True
        elif button == 3:
            if prev == center:
                self._rotation += phase(curr - center)
            else:
                self._rotation += phase((curr - center) / (prev - center))
            self.redraw()
            return True
        else:
            return False

    def on_canvas_button_clicked(self, canvas, button, current):
        if button == 2:
            self._center = self._size / 2
            self._scroll = 0
            self._rotation = 0
            self.redraw()
        return True

    def on_canvas_scroll_event(self, canvas, event):
        self._scroll += event.delta_y
        self.redraw()
        return True

    def on_canvas_button_press_event(self, canvas, event):
        return self._canvas_mouse_listener.on_mouse_button_pressed(canvas, event)

    def on_canvas_button_release_event(self, canvas, event):
        return self._canvas_mouse_listener.on_mouse_button_released(canvas, event)

    def on_canvas_key_press_event(self, window, event):
        print("[canvas] key pressed %d '%s'" % (event.keyval, event.string))
        return True

    def on_canvas_key_release_event(self, window, event):
        print("[canvas] key released %d '%s'" % (event.keyval, event.string))
        return True

    def on_main_window_button_press_event(self, window, event):
        print("[main window] btn pressed %d" % event.button)
        return True

    def on_main_window_button_release_event(self, window, event):
        print("[main window] btn released %d" % event.button)
        return True

    def on_main_window_key_press_event(self, window, event):
        print("[main window] key pressed %d '%s'" % (event.keyval, event.string))
        return False

    def on_main_window_key_release_event(self, window, event):
        print("[main window] key released %d '%s'" % (event.keyval, event.string))
        return False


class InspectorGui(Thread):
    def __init__(self, initial_pose=complex(0, 0), initial_bearing=.0):
        super().__init__(target=self._main, name="inspector-gui", daemon=True)
        # self.canvas = None
        self._controller = None
        # self.window = None
        self._model = InspectorModel(initial_pose * U, initial_bearing)

    def _main(self):
        builder = Gtk.Builder()
        builder.add_from_file(paths(ext='.glade'))

        canvas = builder.get_object("canvas")
        self._controller = InspectorController(canvas, self._model)
        builder.connect_signals(self._controller)

        window = builder.get_object("main_window")
        window.show_all()

        Gtk.main()

    def notify(self, **kwargs):
        GLib.idle_add(lambda: self._notify(**kwargs))

    def _notify(self, **kwargs):
        if 'pose' in kwargs:
            self._model.pose = kwargs['pose'] * U
        if 'bearing' in kwargs:
            self._model.bearing = kwargs['bearing']
        self._controller.redraw()


if __name__ == "__main__":
    t = InspectorGui(100 + 100j, radians(10))
    t.start()

    for x in range(1, 1000):
        t.notify(pose=complex(100 + x, 100 + x), bearing=radians(10 + x))
        time.sleep(1/30)

    t.join()