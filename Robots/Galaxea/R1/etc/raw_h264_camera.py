import pyzed.sl as sl
import sys
import argparse

def main(width, height, fps):
    zed = sl.Camera()
    init = sl.InitParameters()

    if width == 2208 and height == 1242:
        init.camera_resolution = sl.RESOLUTION.HD2K
        print(f"Set resolution to HD2K (2208x1242 per eye, 4416x1242 side-by-side)", file=sys.stderr)
    elif width == 1920 and height == 1080:
        init.camera_resolution = sl.RESOLUTION.HD1080
        print(f"Set resolution to HD1080 (1920x1080 per eye, 3840x1080 side-by-side)", file=sys.stderr)
    elif width == 1280 and height == 720:
        init.camera_resolution = sl.RESOLUTION.HD720
        print(f"Set resolution to HD720 (1280x720 per eye, 2560x720 side-by-side)", file=sys.stderr)
    elif width == 672 and height == 376:
        init.camera_resolution = sl.RESOLUTION.VGA
        print(f"Set resolution to VGA (672x376 per eye, 1344x376 side-by-side)", file=sys.stderr)
    else:
        init.camera_resolution = sl.RESOLUTION.HD2K
        print(f"Unsupported width={width}, height={height}; defaulting to HD2K", file=sys.stderr)

    init.camera_fps = fps
    init.depth_mode = sl.DEPTH_MODE.NONE

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening ZED: {err}", file=sys.stderr)
        sys.exit(1)

    runtime = sl.RuntimeParameters()
    image = sl.Mat()

    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.SIDE_BY_SIDE, sl.MEM.CPU)
            raw_data = image.get_data().tobytes()
            if raw_data:
                sys.stdout.buffer.write(raw_data)
                sys.stdout.flush()
            else:
                print("No raw data retrieved", file=sys.stderr)
        else:
            print("Failed to grab frame", file=sys.stderr)
            break

    zed.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--height', type=int, required=True)
    parser.add_argument('--fps', type=int, default=15)
    args = parser.parse_args()
    main(args.width, args.height, args.fps)