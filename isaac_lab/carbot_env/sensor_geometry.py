"""Sensor geometry and finite observation encoding, independent of Kit."""
import math
import torch

from isaac_sim.carbot_vl53l5cx_mount import vl53l5cx_placements


def rotation_rpy(roll, pitch, yaw, device):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return torch.tensor([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]], device=device)


def lidar_pattern(robot, spec, device):
    """Regular elevation/azimuth proxy; not Livox's non-repetitive scan."""
    mount = robot['sensors']['mid360']
    angles = mount['manufacturer_specs']['vertical_fov_deg']
    elevation = torch.linspace(*angles, spec['channels'], device=device) * math.pi / 180
    azimuth = torch.arange(spec['ray_count'], device=device) * (2*math.pi/spec['ray_count']) - math.pi
    el, az = torch.meshgrid(elevation, azimuth, indexing='ij')
    directions = torch.stack((el.cos()*az.cos(), el.cos()*az.sin(), el.sin()), -1).reshape(-1, 3)
    directions = directions @ rotation_rpy(*mount['rotation_rpy_rad'], device).T
    origin = [*mount['mount_translation_xy_from_base_link_m'],
              mount['housing_top_height_from_ground_m'] - mount['housing_height_m']
              + mount['coordinate_origin_height_from_housing_bottom_m']
              - robot['geometry']['base_link_height_m']]
    return torch.tensor(origin, device=device).expand_as(directions).clone(), directions


def tof_pattern(robot, spec, device):
    """Zone-centre rays, sensor-major then row (top to bottom), column (left to right)."""
    mount = robot['sensors']['vl53l5cx_sim_mount']
    width, height = spec['grid_size']
    x = (2*(torch.arange(width, device=device)+0.5)/width-1) * math.tan(math.radians(mount['detection_fov_deg'][0]/2))
    y = (1-2*(torch.arange(height, device=device)+0.5)/height) * math.tan(math.radians(mount['detection_fov_deg'][1]/2))
    vertical, horizontal = torch.meshgrid(y, -x, indexing='ij')
    directions = torch.stack((torch.ones_like(vertical), horizontal, vertical), -1).reshape(-1, 3)
    directions = torch.nn.functional.normalize(directions, dim=-1)
    starts, rays = [], []
    for _, px, py, pz, yaw in vl53l5cx_placements(robot['geometry']['body_visual_base_plate'], mount):
        rotation = rotation_rpy(0, 0, math.radians(yaw), device)
        optical_offset = rotation @ torch.tensor([mount['board_size_m'][0]/2, 0., 0.], device=device)
        origin = torch.tensor([px, py, pz], device=device) + optical_offset
        starts.append(origin.expand_as(directions))
        rays.append(directions @ rotation.T)
    return torch.cat(starts), torch.cat(rays)


def encode_distances(raw, minimum, maximum):
    """Return metres and validity separately; zero denotes an invalid measurement."""
    valid = torch.isfinite(raw) & (raw >= minimum) & (raw <= maximum)
    return torch.where(valid, raw, torch.zeros_like(raw)), valid.float()
