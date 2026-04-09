import re

with open("sim2sim/play_mujoco.py", "r") as f:
    text = f.read()

# Add --video flag
text = text.replace('parser.add_argument("--duration", type=float, default=10.0)',
                    'parser.add_argument("--duration", type=float, default=10.0)\n    parser.add_argument("--video", type=str, default=None, help="Save video to path")')

# Inject rendering setup
old_setup = """    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data)
    else:
        viewer = None"""

new_setup = """    renderer = None
    frames = []
    if args.video:
        import mediapy as media
        renderer = mujoco.Renderer(model, 480, 640)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        camera.trackbodyid = 0
        camera.distance = 1.5
        camera.azimuth = 90
        camera.elevation = -10
        camera.lookat[:] = [0, 0, 0.4]
    
    if not args.headless and not args.video:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    else:
        viewer = None"""
text = text.replace(old_setup, new_setup)

# Inject rendering inside loop
old_step = """            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break"""

new_step = """            if renderer is not None:
                renderer.update_scene(data, camera)
                frames.append(renderer.render().copy())

            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break"""
text = text.replace(old_step, new_step)

# Inject video save at end
old_end = """    finally:
        if viewer is not None:
            viewer.close()"""

new_end = """    finally:
        if viewer is not None:
            viewer.close()
        if renderer is not None and frames:
            import mediapy as media
            fps = int(1.0 / POLICY_DT)
            print(f"Saving {len(frames)} frames to {args.video} at {fps} FPS...")
            media.write_video(args.video, frames, fps=fps)"""
text = text.replace(old_end, new_end)

with open("sim2sim/play_mujoco.py", "w") as f:
    f.write(text)
print("Patched!")
