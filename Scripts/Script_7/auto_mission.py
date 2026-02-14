"""Main orchestration which loads helper modules from the same folder.

This module uses importlib to load the helper modules so the folder name
may contain spaces (we avoid normal package imports to be robust).
"""
import os
import importlib.util
import time
import cv2


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    base = os.path.dirname(__file__)
    fc = load_module('flight_control', os.path.join(base, 'flight_control.py'))
    viz = load_module('vision', os.path.join(base, 'vision.py'))
    cfg = load_module('config', os.path.join(base, 'config.py'))

    print('Connecting to vehicle...')
    master = fc.connect(cfg.MAV_URI)
    print(f'Connected to system {master.target_system}')

    print('Disabling arming checks and setting parameters...')
    fc.disable_arming_checks(master)
    params = {
        'ACRO_BAL_ROLL': 0.0,
        'ACRO_BAL_PITCH': 0.0,
        'ATC_RAT_RLL_I': 0.18,
        'ATC_RAT_PIT_I': 0.18,
        'ATC_RAT_YAW_I': 0.018,
    }
    fc.set_params(master, params)

    print('Connecting to camera...')
    cam = viz.connect_camera(cfg.HOST, cfg.PORT)

    print('Switching to GUIDED and arming...')
    fc.set_mode(master, 'GUIDED')
    armed = fc.arm(master, timeout=10, force=True)
    if not armed:
        raise RuntimeError('Failed to arm vehicle')

    print('Taking off...')
    fc.takeoff(master, cfg.TAKEOFF_ALT)

    print('Starting autonomous line following...')
    try:
        while True:
            frame_data = viz.read_frame(cam)
            if frame_data is None:
                print('No frame, exiting')
                break
            frame, width, height = frame_data

            error, mask, annotated, found = viz.detect_line(
                frame,
                thresh=cfg.THRESH,
                area_min=cfg.AREA_MIN,
                area_max=cfg.AREA_MAX,
                margin_frac=0.25,
            )

            forward = cfg.FORWARD_SPEED if found else 0.1
            side = -(error * cfg.KP)

            fc.set_velocity_body(master, forward, side, 0)

            cv2.imshow('Annotated', annotated)
            cv2.imshow('Mask', mask)
            if cv2.waitKey(1) & 0xFF == ord('x'):
                break

    except KeyboardInterrupt:
        print('Interrupted')

    print('Cleaning up...')
    try:
        cam.close()
    except Exception:
        pass
    cv2.destroyAllWindows()
    fc.land_and_disarm(master)


if __name__ == '__main__':
    main()
