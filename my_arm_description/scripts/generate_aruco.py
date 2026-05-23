#!/usr/bin/env python3
"""Generate an ArUco marker image with optional white border."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def get_dictionary(name: str):
    try:
        dict_id = getattr(cv2.aruco, name)
    except AttributeError as exc:
        raise ValueError(f"unknown dictionary: {name}") from exc

    try:
        return cv2.aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        return cv2.aruco.Dictionary_get(dict_id)


def generate_marker(dictionary, marker_id: int, side_px: int) -> np.ndarray:
    try:
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    except AttributeError:
        return cv2.aruco.drawMarker(dictionary, marker_id, side_px)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate an ArUco marker image")
    parser.add_argument('--dict', default='DICT_6X6_250', help='OpenCV ArUco dictionary name')
    parser.add_argument('--id', type=int, default=2, help='Marker id')
    parser.add_argument('--side-px', type=int, default=600, help='Marker side length in pixels')
    parser.add_argument('--border-px', type=int, default=60, help='White border size in pixels (0 = no border)')
    parser.add_argument('--output', default='aruco_marker.png', help='Output PNG path')
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        dictionary = get_dictionary(args.dict)
    except Exception as exc:
        print(f"invalid dictionary: {exc}", file=sys.stderr)
        return 2

    if args.id < 0:
        print("marker id must be >= 0", file=sys.stderr)
        return 2
    if args.side_px <= 0:
        print("side-px must be > 0", file=sys.stderr)
        return 2
    if args.border_px < 0:
        print("border-px must be >= 0", file=sys.stderr)
        return 2

    marker = generate_marker(dictionary, args.id, args.side_px)

    if args.border_px > 0:
        size = args.side_px + 2 * args.border_px
        img = np.ones((size, size), dtype=np.uint8) * 255
        img[args.border_px:args.border_px + args.side_px, args.border_px:args.border_px + args.side_px] = marker
    else:
        img = marker

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), img):
        print("failed to write output image", file=sys.stderr)
        return 1

    print(f"wrote {output} ({img.shape[1]}x{img.shape[0]} px)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
