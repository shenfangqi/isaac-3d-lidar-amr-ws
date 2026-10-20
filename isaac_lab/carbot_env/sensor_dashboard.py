"""Live Kit UI for inspecting the exact sensor buffers used by the policy."""
import math
from collections import deque

import numpy as np
import torch
import omni.ui as ui
from isaaclab.utils.math import quat_apply
from isaacsim.util.debug_draw import _debug_draw

from .sensor_geometry import encode_distances
from .spec import load_environment_spec

TOF_NAMES = ('Front L', 'Front R', 'Rear L', 'Rear R',
             'Left F', 'Left R', 'Right F', 'Right R')


def heatmap(raw, minimum, maximum, scale=1, grid=False):
    """Fixed metre scale: warm near, cyan far, dark for invalid values."""
    raw = np.asarray(raw)
    valid = np.isfinite(raw) & (raw >= minimum) & (raw <= maximum)
    amount = np.clip((np.where(valid, raw, minimum)-minimum)/(maximum-minimum), 0, 1)
    near, far = np.array([255, 128, 70]), np.array([65, 204, 235])
    rgb = near[None, None, :] * (1-amount[..., None]) + far[None, None, :] * amount[..., None]
    rgb[~valid] = (12, 17, 25)
    rgba = np.concatenate((rgb.astype(np.uint8), np.full((*raw.shape, 1), 255, dtype=np.uint8)), axis=-1)
    rgba = np.repeat(np.repeat(rgba, scale, axis=0), scale, axis=1)
    if grid and scale > 1:
        rgba[::scale, :, :3] = 30
        rgba[:, ::scale, :3] = 30
    return np.ascontiguousarray(rgba)


def summary(raw, minimum, maximum):
    valid = raw[np.isfinite(raw) & (raw >= minimum) & (raw <= maximum)]
    if not valid.size:
        return 'No valid returns'
    return f'{valid.min():.2f} - {valid.max():.2f} m | {valid.size}/{raw.size} valid'


class SensorDashboard:
    def __init__(self, env):
        self.env = env
        self.spec = load_environment_spec()['observations']
        self.commands = deque()
        self.selected_tof = 0
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.window = ui.Window('Carbot | Sensor Check', width=530, height=970)
        self.window.frame.style = {
            'Label': {'font_size': 14, 'color': 0xffe9e3d8},
            'Button': {'background_color': 0xff44362b, 'border_radius': 4, 'margin': 2},
            'Button:hovered': {'background_color': 0xff65533b},
        }
        self.depth_provider = ui.ByteImageProvider()
        self.tof_providers = [ui.ByteImageProvider() for _ in range(8)]
        self.tof_labels = []
        with self.window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=6, height=0):
                    ui.Label('CARBOT / LIVE SENSOR CHECK', height=26, style={'font_size': 19})
                    self.status = ui.Label('Warming up sensors...', height=22)
                    ui.Label('Move / rotate the simulated robot; watch all three sensors.', height=20)
                    with ui.HStack(height=32):
                        ui.Button('Forward 10 cm', clicked_fn=lambda: self.commands.append(('move', .1)))
                        ui.Button('Back 10 cm', clicked_fn=lambda: self.commands.append(('move', -.1)))
                        ui.Button('Reset pose', clicked_fn=lambda: self.commands.append(('reset', 0)))
                    with ui.HStack(height=32):
                        ui.Button('Turn left 15 deg', clicked_fn=lambda: self.commands.append(('turn', math.pi/12)))
                        ui.Button('Turn right 15 deg', clicked_fn=lambda: self.commands.append(('turn', -math.pi/12)))
                        ui.Button('ToF mounts', clicked_fn=self.mounts_view)
                        ui.Button('Overview', clicked_fn=self.overview)
                    ui.Label('Steps change pose directly. +X arrow = vehicle front.', height=20)
                    ui.Separator(height=4)
                    ui.Label('D455 DEPTH   /   32 x 24   /   pitch +15 deg', height=24)
                    with ui.HStack(height=144):
                        ui.Spacer()
                        ui.ImageWithProvider(self.depth_provider, width=192, height=144)
                        ui.Spacer()
                    self.depth_label = ui.Label('', height=20)
                    ui.Label('D455: orange 0.6 m -> cyan 6 m | dark = invalid', height=20)
                    ui.Separator(height=4)
                    ui.Label('VL53L5CX   /   8 modules x 8 x 8 zones', height=24)
                    for row in range(2):
                        with ui.HStack(height=132, spacing=4):
                            for index in range(row*4, row*4+4):
                                with ui.VStack(spacing=3):
                                    ui.Button(TOF_NAMES[index], height=26,
                                              clicked_fn=lambda i=index: self.select_tof(i))
                                    with ui.HStack(height=80):
                                        ui.Spacer()
                                        ui.ImageWithProvider(self.tof_providers[index], width=80, height=80)
                                        ui.Spacer()
                                    self.tof_labels.append(ui.Label('-- m', height=22,
                                        alignment=ui.Alignment.CENTER, style={'font_size': 12}))
                    ui.Label('ToF: orange 0.02 m -> cyan 4 m | dark = invalid', height=20)
                    self.selection_label = ui.Label('Selected rays: Front L (orange)', height=20)
                    ui.Separator(height=4)
                    ui.Label('MID-360   /   8 elevations x 72 directions', height=24)
                    self.lidar_label = ui.Label('', height=20)
                    with ui.HStack(height=22):
                        self.show_lidar = ui.CheckBox(width=22)
                        self.show_lidar.model.set_value(True)
                        ui.Label('Cyan points: MID-360 hits')
                        self.show_tof = ui.CheckBox(width=22)
                        self.show_tof.model.set_value(True)
                        ui.Label('Orange points: ToF hits')
                    ui.Label('Static geometry preview. Invalid is not verified free space.', height=22)
        self.arrange_layout()
        self.mounts_view()

    def arrange_layout(self):
        for title in ('Stage', 'Layer', 'Render Settings', 'Content', 'Console',
                      'Isaac Sim Assets', 'Isaac Sim Assets [Beta]', 'IsaacLab',
                      'Property', 'Simulation Settings', 'Semantics Schema Editor'):
            window = ui.Workspace.get_window(title)
            if window:
                window.visible = False
        viewport = ui.Workspace.get_window('Viewport')
        if viewport:
            self.window.dock_in(viewport, ui.DockPosition.RIGHT, .39)

    def select_tof(self, index):
        self.selected_tof = index
        self.selection_label.text = f'Selected rays: {TOF_NAMES[index]} (orange)'

    def overview(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=(2.4, 2.8, 3.2), target=(0., 0., .1))

    def mounts_view(self):
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=(.52, .58, .36), target=(0., 0., .065))

    def apply_commands(self):
        robot = self.env.scene['robot']
        while self.commands:
            command, value = self.commands.popleft()
            state = robot.data.root_state_w.clone()
            if command == 'reset':
                state = robot.data.default_root_state.clone()
                state[:, :3] += self.env.scene.env_origins
            else:
                q = state[0, 3:7]
                yaw = math.atan2(float(2*(q[0]*q[3]+q[1]*q[2])), float(1-2*(q[2]**2+q[3]**2)))
                if command == 'move':
                    state[0, 0] += value*math.cos(yaw)
                    state[0, 1] += value*math.sin(yaw)
                    # Keep the inspection robot inside the four reference walls.
                    state[0, 0].clamp_(-.85, .55)
                    state[0, 1].clamp_(-.75, .75)
                elif command == 'turn':
                    yaw += value
                    state[0, 3:7] = torch.tensor([math.cos(yaw/2), 0, 0, math.sin(yaw/2)], device=self.env.device)
            state[:, 7:] = 0
            robot.write_root_state_to_sim(state)
            self.env.action_manager.reset()
            self.env.scene.reset()

    def update(self):
        self.status.text = f'LIVE   |   simulation {self.env.sim.current_time:.2f} s   |   units: metres'
        camera = self.env.scene['d455']
        raw = camera.data.output['distance_to_image_plane'][0, ..., 0].cpu().numpy()
        d = self.spec['d455']
        image = heatmap(raw, d['minimum_range_m'], d['maximum_range_m'], scale=8)
        self.depth_provider.set_bytes_data(image.ravel().data, [image.shape[1], image.shape[0]])
        self.depth_label.text = summary(raw, d['minimum_range_m'], d['maximum_range_m'])
        self.draw.clear_points()
        self.draw.clear_lines()
        for name, toggle, color in [('lidar', self.show_lidar, (0.2, .85, 1., 1.)),
                                     ('tof', self.show_tof, (1., .45, .12, 1.))]:
            sensor = self.env.scene[name]
            data = sensor.data
            quat = data.quat_w[0].expand(sensor.num_rays, -1)
            starts = data.pos_w[0] + quat_apply(quat, sensor.ray_starts[0])
            hits = data.ray_hits_w[0]
            distance = (hits-starts).norm(dim=-1)
            settings = self.spec[name]
            values, valid = encode_distances(distance, settings['minimum_range_m'], settings['maximum_range_m'])
            raw = distance.cpu().numpy()
            if name == 'lidar':
                self.lidar_label.text = summary(raw, settings['minimum_range_m'], settings['maximum_range_m'])
            else:
                for i, grid in enumerate(raw.reshape(8, 8, 8)):
                    image = heatmap(grid, settings['minimum_range_m'], settings['maximum_range_m'], scale=12, grid=True)
                    self.tof_providers[i].set_bytes_data(image.ravel().data, [96, 96])
                    measured = grid[np.isfinite(grid) & (grid >= settings['minimum_range_m']) & (grid <= settings['maximum_range_m'])]
                    self.tof_labels[i].text = f'min {measured.min():.2f} m' if measured.size else 'invalid'
            if toggle.model.as_bool:
                points = hits[valid.bool()].cpu().tolist()
                self.draw.draw_points(points, [color]*len(points), [4.]*len(points))
                if name == 'tof':
                    indices = torch.arange(self.selected_tof*64, (self.selected_tof+1)*64, device=self.env.device)
                    indices = indices[valid[indices].bool()]
                    self.draw.draw_lines(starts[indices].cpu().tolist(), hits[indices].cpu().tolist(),
                                         [color]*len(indices), [1.]*len(indices))
        # Viewport-only arrow. DebugDraw does not add geometry to the sensor mesh.
        robot = self.env.scene['robot']
        origin = robot.data.root_pos_w[0].clone()
        origin[2] += .32
        arrow = torch.tensor([[0.,0.,0.],[.3,0.,0.],[.23,.04,0.],[.23,-.04,0.]], device=self.env.device)
        arrow = origin + quat_apply(robot.data.root_quat_w[0].expand(4,-1), arrow)
        self.draw.draw_lines(arrow[[0,1,1]].cpu().tolist(), arrow[[1,2,3]].cpu().tolist(),
                             [(0.4,1.,0.35,1.)]*3, [3.]*3)

    def close(self):
        self.draw.clear_points()
        self.draw.clear_lines()
        self.window.destroy()
