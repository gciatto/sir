import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib
from gi.repository import cairo
from gciatto.gi.mouse import HighLevelMouseObservable
from functools import lru_cache
from cmath import phase
from res import paths

U = 50

@lru_cache()
def zoom(x):
    return 2 ** (x / 50)


class InspectorGui:
    def __init__(self, canvas):
        super().__init__()
        self.canvas = canvas
        self.size = complex(0, 0)
        self.center = complex(0, 0)
        self.scroll = 0
        self.canvas_mouse_listener = HighLevelMouseObservable(
            drag=self.on_canvas_drag_notify_event,
            released=self.on_canvas_button_clicked
        )
        self.rotation = 0

    def _redraw(self):
        self.canvas.queue_draw()
        return True

    def _get_mouse_pressed(self):
        return next((k for k, v in self.mouse_pressed.items() if v), None)

    def on_main_window_delete_event(self, *args):
        Gtk.main_quit(*args)

    def on_canvas_draw(self, canvas, cc: cairo.Context):
        print("[canvas] draw")

        cc.translate(self.center.real, self.center.imag)
        zoom_factor = zoom(self.scroll)
        cc.scale(zoom_factor, -zoom_factor)
        cc.rotate(-self.rotation)

        cc.move_to(-U, 0)
        cc.line_to(10 * U, 0)
        cc.move_to(0, -U)
        cc.line_to(0, 10 * U)
        cc.stroke()
        return True

    def on_canvas_size_allocate(self, canvas, rect):
        self.size = complex(rect.width, rect.height)
        self.center = self.size / 2
        print("[canvas] resize: %sx%s" % (rect.width, rect.height))

    def on_canvas_motion_notify_event(self, canvas, event):
        return self.canvas_mouse_listener.on_mouse_movement(canvas, event)

    def on_canvas_drag_notify_event(self, canvas, button, curr, prev, initial):
        print("[canvas] drag: (%s; %s)" % (prev.real, prev.imag))
        center = self.center
        if button == 1:
            self.center += curr - prev
            self._redraw()
            return True
        elif button == 3:
            if prev == center:
                self.rotation += phase(curr - center)
            else:
                self.rotation += phase((curr - center) / (prev - center))
            self._redraw()
            return True
        else:
            return False

    def on_canvas_button_clicked(self, canvas, button, current):
        if button == 2:
            self.center = self.size / 2
            self.scroll = 0
            self.rotation = 0
            self._redraw()
        return True

    def on_canvas_scroll_event(self, canvas, event):
        self.scroll += event.delta_y
        print("[canvas] scroll: (%s; %s)" % (event.delta_x, event.delta_y))
        self._redraw()
        return True

    def on_canvas_button_press_event(self, canvas, event):
        return self.canvas_mouse_listener.on_mouse_button_pressed(canvas, event)

    def on_canvas_button_release_event(self, canvas, event):
        return self.canvas_mouse_listener.on_mouse_button_released(canvas, event)

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


builder = Gtk.Builder()
builder.add_from_file(paths(ext='.glade'))

canvas = builder.get_object("canvas")

builder.connect_signals(InspectorGui(canvas))

window = builder.get_object("main_window")
window.show_all()

Gtk.main()