def do_nothing(*args, **kwargs):
    pass


class HighLevelMouseObservable:

    def __init__(self, movement=do_nothing, drag=do_nothing, pressed=do_nothing, released=do_nothing):
        self._mouse_pressed = dict()
        self._drag_begin_position = dict()
        self._mouse_position = complex(0, 0)

        self.set_callbacks(movement, drag, pressed, released)

    def set_callbacks(self, movement=None, drag=None, pressed=None, released=None):
        if movement:
            self._notify_movement = movement
        if drag:
            self._notify_drag = drag
        if pressed:
            self._notify_pressed = pressed
        if released:
            self._notify_released = released

    def _get_mouse_pressed(self):
        return next((k for k, v in self._mouse_pressed.items() if v), None)

    def on_mouse_movement(self, src, event):
        pressed = self._get_mouse_pressed()
        old = self._mouse_position
        new = complex(event.x, event.y)
        self._mouse_position = new
        if pressed is None:
            self._notify_movement(src, new, old)
        else:
            self._notify_drag(src, pressed, new, old, self._drag_begin_position[pressed])
        return True

    def on_mouse_button_pressed(self, src, event):
        pressed = event.button
        self._mouse_pressed[pressed] = True
        position = complex(event.x, event.y)
        self._mouse_position = position
        self._drag_begin_position[pressed] = position
        self._notify_pressed(src, pressed, position)
        return True

    def on_mouse_button_released(self, src, event):
        pressed = event.button
        position = complex(event.x, event.y)
        self._mouse_pressed[pressed] = False
        self._notify_released(src, pressed, position)
        return True
