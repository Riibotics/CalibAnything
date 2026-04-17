#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np

import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract a minimal CalibAnything dataset from a ROS 2 bag."
    )
    parser.add_argument(
        "--bag-path",
        default="./data/rosbag2_2026_04_17-10_29_33_0.db3",
        help="Path to a rosbag2 directory. If a .db3/.mcap file is given, its parent directory is used.",
    )
    parser.add_argument("--lidar-topic", default="/fork_lidar")
    parser.add_argument("--image-topic", default="/fork_camera/rgb")
    parser.add_argument("--cam-info-topic", default="/fork_camera/rgb_info")
    parser.add_argument("--output-dir", default="./dataset2")
    parser.add_argument(
        "--pc-duration-sec",
        type=float,
        default=2.0,
        help="Seconds of lidar data to accumulate into one point cloud.",
    )
    parser.add_argument(
        "--threads",
        type=int,
        default=min(4, os.cpu_count() or 4),
        help="Thread count written into calib.json.",
    )
    return parser.parse_args()


def resolve_bag_uri(raw_path: str) -> tuple[Path, str]:
    """Return (uri, storage_id) for rosbag2_py.StorageOptions.

    If a .db3 / .mcap file is given directly we use it as-is so that the
    reader does NOT fall back to the directory's metadata.yaml (which may
    reference a different file).
    """
    bag_path = Path(raw_path).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    if bag_path.is_file():
        if bag_path.suffix == ".db3":
            return bag_path, "sqlite3"
        if bag_path.suffix == ".mcap":
            return bag_path, "mcap"

    # Directory bag (contains metadata.yaml)
    return bag_path, ""


def make_output_dirs(output_dir: Path):
    for folder in ("images", "pc", "masks", "processed_masks"):
        (output_dir / folder).mkdir(parents=True, exist_ok=True)


def save_ascii_pcd(points, output_path: Path):
    with output_path.open("w", encoding="utf-8") as file_obj:
        file_obj.write(
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
        )
        file_obj.write(f"WIDTH {len(points)}\n")
        file_obj.write("HEIGHT 1\n")
        file_obj.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        file_obj.write(f"POINTS {len(points)}\n")
        file_obj.write("DATA ascii\n")
        for point in points:
            file_obj.write(
                f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f} {point[3]:.4f}\n"
            )


def image_msg_to_bgr(msg):
    encoding = msg.encoding.lower()
    dtype = np.uint8

    if encoding in {"mono16", "16uc1", "16sc1"}:
        dtype = np.uint16

    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8uc4": 4,
        "mono16": 1,
        "16uc1": 1,
        "16sc1": 1,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    itemsize = np.dtype(dtype).itemsize
    row_width = msg.width * channels
    if msg.step < row_width * itemsize:
        raise ValueError(
            f"Image step is smaller than expected for encoding {msg.encoding}: "
            f"step={msg.step}, expected>={row_width * itemsize}"
        )

    data = np.frombuffer(msg.data, dtype=dtype)
    data = data.reshape(msg.height, msg.step // itemsize)
    data = data[:, :row_width]

    if channels == 1:
        image = data.reshape(msg.height, msg.width)
    else:
        image = data.reshape(msg.height, msg.width, channels)

    if msg.is_bigendian and itemsize > 1:
        image = image.byteswap().newbyteorder()

    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if channels == 1:
        if dtype == np.uint16:
            image = cv2.convertScaleAbs(image, alpha=255.0 / 65535.0)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    return image


def build_calib_config(cam_k, cam_dist, thread_count):
    k_matrix = [
        [cam_k[0], cam_k[1], cam_k[2]],
        [cam_k[3], cam_k[4], cam_k[5]],
        [cam_k[6], cam_k[7], cam_k[8]],
    ]

    return {
        "cam_K": {"rows": 3, "cols": 3, "data": k_matrix},
        "cam_dist": {"cols": len(cam_dist), "data": cam_dist},
        "T_lidar_to_cam": {
            "rows": 4,
            "cols": 4,
            "data": [
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        "T_lidar_to_cam_gt": {"available": False},
        "img_folder": "/images",
        "mask_folder": "/processed_masks",
        "pc_folder": "/pc",
        "img_format": ".png",
        "pc_format": ".pcd",
        "file_name": ["000000"],
        "params": {
            "search_range": {"rot_deg": 5.0, "trans_m": 0.25},
            "min_plane_point_num": 250,
            "cluster_tolerance": 0.1,
            "point_range": {"top": 0.12, "bottom": 0.88},
            "search_num": 1000,
            "thread": {"is_multi_thread": thread_count > 1, "num_thread": thread_count},
            "down_sample": {"is_valid": True, "voxel_m": 0.04},
            "segmentation": {
                "plane_distance_threshold_m": 0.1,
                "normal_k_search": 30,
                "euclidean_min_cluster_size": 150,
            },
        },
    }


def main():
    args = parse_args()

    output_dir = Path(args.output_dir).expanduser().resolve()
    bag_uri, storage_id = resolve_bag_uri(args.bag_path)
    make_output_dirs(output_dir)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    type_map = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required_topics = [args.lidar_topic, args.image_topic, args.cam_info_topic]
    missing_topics = [topic for topic in required_topics if topic not in type_map]
    if missing_topics:
        raise RuntimeError(f"Required topics are missing from bag: {missing_topics}")

    cam_k = None
    cam_dist = None
    fallback_image = None
    synced_image = None

    lidar_start_time = None
    accumulated_points = []

    print(f"Reading ROS 2 bag: {bag_uri}")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in required_topics:
            continue

        t_sec = timestamp / 1e9
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == args.cam_info_topic and cam_k is None:
            cam_k = list(getattr(msg, "k", getattr(msg, "K", [])))
            cam_dist = list(getattr(msg, "d", getattr(msg, "D", [])))
            print("Camera info extracted.")

        if topic == args.image_topic:
            cv_img = image_msg_to_bgr(msg)
            if fallback_image is None:
                fallback_image = cv_img

            if (
                lidar_start_time is not None
                and synced_image is None
                and 0.0 <= t_sec - lidar_start_time <= args.pc_duration_sec
            ):
                synced_image = cv_img
                print("Selected an image captured during the lidar accumulation window.")

        if topic == args.lidar_topic:
            if lidar_start_time is None:
                lidar_start_time = t_sec

            if t_sec - lidar_start_time <= args.pc_duration_sec:
                for point in pc2.read_points(
                    msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
                ):
                    accumulated_points.append((point[0], point[1], point[2], point[3]))

        if (
            cam_k is not None
            and fallback_image is not None
            and lidar_start_time is not None
            and t_sec - lidar_start_time > args.pc_duration_sec
        ):
            break

    if cam_k is None or cam_dist is None:
        raise RuntimeError("Camera intrinsics were not found in the bag.")
    if not accumulated_points:
        raise RuntimeError("No lidar points were collected from the bag.")

    image_to_save = synced_image if synced_image is not None else fallback_image
    if image_to_save is None:
        raise RuntimeError("No camera image was found in the bag.")
    if synced_image is None:
        print(
            "Warning: no image was found in the accumulation window, "
            "using the first available image."
        )

    image_path = output_dir / "images" / "000000.png"
    pcd_path = output_dir / "pc" / "000000.pcd"
    calib_path = output_dir / "calib.json"

    cv2.imwrite(str(image_path), image_to_save)
    save_ascii_pcd(accumulated_points, pcd_path)

    calib_config = build_calib_config(cam_k, cam_dist, max(args.threads, 1))
    with calib_path.open("w", encoding="utf-8") as file_obj:
        json.dump(calib_config, file_obj, indent=4)

    print(f"Saved image to: {image_path}")
    print(
        f"Saved accumulated point cloud ({len(accumulated_points)} points) to: {pcd_path}"
    )
    print(f"Generated config: {calib_path}")
    print("Review T_lidar_to_cam and search_range in calib.json before running calibration.")


if __name__ == "__main__":
    main()
