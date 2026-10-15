"""Isaac Lab 2.2 static scene ray casting with explicit clone isolation.

Each environment gets its own world-space mesh. Ground and all static Mesh/Cube
geometry under that environment are included; the robot is excluded. Moving
obstacles are rejected because baked geometry cannot follow their motion.
"""
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.sensors import RayCaster
from isaaclab.utils.warp import convert_to_warp_mesh


class StaticSceneRayCaster(RayCaster):
    def _initialize_warp_meshes(self):
        stage = sim_utils.get_current_stage()
        parents = sim_utils.find_matching_prim_paths(self.cfg.prim_path)
        self._environment_meshes = []
        for parent in parents:
            env_path = parent.split('/Robot/')[0]
            vertices, triangles = [], []
            count = 0
            for root in ['/World/ground', env_path]:
                prim = stage.GetPrimAtPath(root)
                if not prim.IsValid():
                    raise RuntimeError(f'Missing sensor geometry root: {root}')
                children = list(Usd.PrimRange(prim))
                planes = [p for p in children if p.GetTypeName() == 'Plane']
                # The stock ground also has a million-metre visual mesh. Casting
                # that mesh causes float32 near-ground errors; use its physics
                # plane only, locally tessellated around this clone.
                if root == '/World/ground' and planes:
                    children = planes
                for child in children:
                    path = str(child.GetPath())
                    if '/Robot' in path:
                        continue
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        raise RuntimeError(f'Static sensor proxy does not support moving obstacle: {path}')
                    kind = child.GetTypeName()
                    if kind not in ('Mesh', 'Cube', 'Plane'):
                        if child.IsA(UsdGeom.Gprim):
                            raise RuntimeError(f'Convert sensor obstacle to a triangle mesh: {path} ({kind})')
                        continue
                    if kind == 'Mesh':
                        mesh = UsdGeom.Mesh(child)
                        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
                        indices = list(mesh.GetFaceVertexIndicesAttr().Get())
                        faces, offset = [], 0
                        for n in mesh.GetFaceVertexCountsAttr().Get():
                            for j in range(1, n-1):
                                faces.append([indices[offset], indices[offset+j], indices[offset+j+1]])
                            offset += n
                    elif kind == 'Cube':
                        h = UsdGeom.Cube(child).GetSizeAttr().Get()/2
                        points = np.array([[-h,-h,-h],[h,-h,-h],[h,h,-h],[-h,h,-h],
                                           [-h,-h,h],[h,-h,h],[h,h,h],[-h,h,h]])
                        faces = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                                 [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
                    else:
                        points, faces = None, [[0,1,2],[0,2,3]]
                    transform = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    if kind == 'Plane':
                        centre = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(env_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
                        origin = np.array(transform.ExtractTranslation())
                        axis = UsdGeom.Plane(child).GetAxisAttr().Get()
                        axes = {'X': ((0,1,0),(0,0,1)), 'Y': ((0,0,1),(1,0,0)), 'Z': ((1,0,0),(0,1,0))}
                        u, v = [np.array(transform.TransformDir(Gf.Vec3d(*a))) for a in axes[axis]]
                        u, v = u/np.linalg.norm(u), v/np.linalg.norm(v)
                        normal = np.cross(u, v)
                        centre -= normal*np.dot(centre-origin, normal)
                        extent = max(100., 4*self.cfg.max_distance)
                        points = np.array([centre+extent*(a*u+b*v) for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])
                    else:
                        points = np.array([transform.Transform(Gf.Vec3d(*p)) for p in points])
                    vertices.append(points)
                    triangles.append(np.asarray(faces) + count)
                    count += len(points)
            if not vertices:
                raise RuntimeError(f'No static sensor geometry for {env_path}')
            self._environment_meshes.append(convert_to_warp_mesh(
                np.concatenate(vertices), np.concatenate(triangles), device=self.device))
        self.meshes[self.cfg.mesh_prim_paths[0]] = self._environment_meshes[0]

    def _update_buffers_impl(self, env_ids):
        # RayCaster 2.2 supports only one mesh per call. Batch all rays belonging
        # to one clone against that clone's mesh to prevent cross-environment hits.
        for index in env_ids:
            idx = int(index)
            self.meshes[self.cfg.mesh_prim_paths[0]] = self._environment_meshes[idx]
            super()._update_buffers_impl([idx])
