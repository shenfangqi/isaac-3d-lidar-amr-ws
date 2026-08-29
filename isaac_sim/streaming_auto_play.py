"""Load the project warehouse and play it inside an existing streaming Kit app."""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd


USD_PATH = (
    "/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/usd/"
    "warehouse_3d_nav_origin_carter.usd"
)


async def load_and_play() -> None:
    print(f"Opening streaming USD: {USD_PATH}", flush=True)
    omni.usd.get_context().open_stage(USD_PATH)

    app = omni.kit.app.get_app()
    for _ in range(500):
        await app.next_update_async()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Streaming warehouse USD stage failed to load")

    print(f"Streaming stage loaded: {stage.GetRootLayer().realPath}", flush=True)
    omni.timeline.get_timeline_interface().play()

    for _ in range(30):
        await app.next_update_async()

    print("Streaming warehouse timeline started.", flush=True)


asyncio.ensure_future(load_and_play())
