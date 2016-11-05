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
from math import radians, atan2
from gciatto.stat.normal import error_ellipse
from numpy import zeros, eye, matrix
from random import gauss
from gciatto.utils import arc_cos_sin
from sir.client.strategies.ekf import landmarks


ErrorEllipse = namedtuple('ErrorEllipse', ['h_axis', 'v_axis', 'rotation'])


class LandmarkAdapter:
    def __init__(self, position=zeros(2), covariances=zeros((2, 2))):
        self.pose = complex(*position)
        sizes, axes = error_ellipse(covariances)
        angle = arc_cos_sin(axes[0, 0], axes[1, 0])
        self.error_ellipse = ErrorEllipse(sizes[0], sizes[1], angle)


class StateAdapter(LandmarkAdapter):
    def __init__(self, position=zeros(3), covariances=zeros((3, 3))):
        super().__init__(position[0:2], covariances[0:2, 0:2])
        self.bearing = position[2]
        self.angle_error = covariances[2, 2]


class InspectorModel:
    def __init__(self, initial_pose=complex(0, 0), initial_bearing=.0):
        self.origin = initial_pose
        self.orientation = initial_bearing
        self.pose = initial_pose
        self.bearing = initial_bearing
        self.state = StateAdapter()
        self.landmarks = []


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
        return True

    def _draw_world(self, canvas, cc: cairo.Context):
        cc.translate(self._center.real, self._center.imag)
        zoom_factor = zoom(self._scroll)
        cc.scale(zoom_factor, -zoom_factor)
        cc.rotate(-self._rotation)

        draw_axes(canvas, cc)

        self._draw_frame(canvas, cc)

        self._draw_actual_robot(canvas, cc)

    def _draw_frame(self, canvas, cc):
        cc.save()
        cc.set_source_rgb(0, 0, 1)

        origin = self._model.origin * U

        cc.translate(origin.real, origin.imag)
        cc.rotate(self._model.orientation)

        draw_axes(canvas, cc)

        self._draw_expected_robot(canvas, cc)
        self._draw_landmarks(canvas, cc)

        cc.restore()

    def _draw_actual_robot(self, canvas, cc):
        cc.save()
        cc.set_source_rgb(0, 0.5, 0)
        pose = self._model.pose * U
        cc.translate(pose.real, pose.imag)
        cc.rotate(self._model.bearing)

        draw_active(canvas, cc)
        cc.restore()

    def _draw_expected_robot(self, canvas, cc):
        cc.save()
        cc.set_source_rgba(0, 1, 0, 0.2)
        state = self._model.state
        pose = state.pose * U
        cc.translate(pose.real, pose.imag)

        fill_ellipse(canvas, cc, state.error_ellipse.h_axis * 3, state.error_ellipse.v_axis * 3,
                     state.error_ellipse.rotation, U)

        cc.rotate(state.bearing)

        fill_arc(canvas, cc, state.angle_error)

        cc.set_source_rgba(0, 1, 0, 1)
        draw_active(canvas, cc)

        cc.restore()

    def _draw_landmarks(self, canvas, cc):
        for lmk in self._model.landmarks:
            cc.save()
            cc.set_source_rgba(1, 0, 0, 0.2)
            pose = lmk.pose * U
            cc.translate(pose.real, pose.imag)

            fill_ellipse(canvas, cc, lmk.error_ellipse.h_axis * 3, lmk.error_ellipse.v_axis * 3,
                         lmk.error_ellipse.rotation, U)

            cc.set_source_rgba(1, 0, 0, 1)
            draw_passive(canvas, cc)
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
        self._model = InspectorModel(initial_pose, initial_bearing)

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
        if 'pose' in kwargs and 'bearing' in kwargs:
            self._model.pose = kwargs['pose']
            self._model.bearing = kwargs['bearing']
        if 'state' in kwargs and 'covariances' in kwargs:
            state = kwargs['state']
            covs = kwargs['covariances']
            self._model.state = StateAdapter(state[0:3], covs[0:3, 0:3])
            self._model.landmarks = [LandmarkAdapter(lmk_pos, lmk_covs) for lmk_i, lmk_pos, lmk_covs in landmarks(state, covs)]

        self._controller.redraw()


if __name__ == "__main__":
    pos_fac = 0.01
    init_pos = 1 + 1j
    init_rot = radians(10)
    t = InspectorGui(init_pos, init_rot)
    t.start()

    import time

    for x in range(1, 1000):
        p = init_pos + complex(x, x) * pos_fac
        r = init_rot + radians(x)

        s = (gauss(p.real, pos_fac), gauss(p.imag, pos_fac), gauss(r, radians(1)))
        t.notify(
            pose=p,
            bearing=r,
            state=s,
            covariances=matrix([
                [0.5, 1, 0],
                [1, 0.25, 0],
                [0, 0, radians(20)]
            ])
        )
        time.sleep(1 / 30)

    t.join()
