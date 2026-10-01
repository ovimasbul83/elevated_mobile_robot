"""Keyboard mecanum drive for a four-wheel X roller configuration."""
import threading


class WheelControl:
    def __init__(self, model, speed=.10, turn_speed=.25):
        self.speed, self.turn_speed = speed, turn_speed
        self.radius = float(model.geom('wheel_fl_geom').size[0])
        self.half_track = abs(float(model.body('wheel_module_fl').pos[1] + model.body('wheel_fl').pos[1]))
        self.half_wheelbase = abs(float(model.body('wheel_module_fl').pos[0]))
        self.motors = {c: model.actuator('wheel_'+c+'_motor').id for c in ('fl', 'fr', 'bl', 'br')}
        self.linear, self.lateral, self.yaw = 0., 0., 0.
        self.target = (0., 0., 0.)
        self.revision = 0
        self.lock = threading.Lock()

    def set_motion(self, motion):
        commands = {'stop': (0., 0., 0.), 'forward': (self.speed, 0., 0.),
                    'backward': (-self.speed, 0., 0.), 'left': (0., 0., self.turn_speed),
                    'right': (0., 0., -self.turn_speed),
                    'strafe-left': (0., self.speed, 0.), 'strafe-right': (0., -self.speed, 0.)}
        with self.lock:
            self.target = commands[motion]
            self.revision += 1

    def key_callback(self, key):
        # MuJoCo supplies key-press events, not key-release events: commands latch.
        motion = {ord('W'): 'forward', ord('S'): 'backward', ord('A'): 'left',
                  ord('D'): 'right', ord('Q'): 'strafe-left', ord('E'): 'strafe-right', ord('X'): 'stop'}.get(key)
        if motion:
            self.set_motion(motion)
            print(f'Drive: {motion}', flush=True)

    def update(self, data, dt):
        with self.lock:
            linear_target, lateral_target, yaw_target = self.target
        # Gentle acceleration; X ramps to a stop instead of teleporting velocity.
        self.linear += max(-.15*dt, min(.15*dt, linear_target-self.linear))
        self.lateral += max(-.15*dt, min(.15*dt, lateral_target-self.lateral))
        self.yaw += max(-.4*dt, min(.4*dt, yaw_target-self.yaw))
        turn = (self.half_wheelbase+self.half_track)*self.yaw
        rates = {'fl': self.linear-self.lateral-turn,
                 'fr': self.linear+self.lateral+turn,
                 'bl': self.linear+self.lateral-turn,
                 'br': self.linear-self.lateral+turn}
        for corner, actuator in self.motors.items():
            data.ctrl[actuator] = rates[corner]/self.radius
