#!/usr/bin/env python3
"""Generate a printable chessboard calibration target."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def parse_size(text: str) -> (int, int):
    if 'x' not in text:
        raise ValueError("size must be like 8x6")
    cols_s, rows_s = text.lower().split('x', 1)
    cols = int(cols_s)
    rows = int(rows_s)
    if cols <= 0 or rows <= 0:
        raise ValueError("size must be positive")
    return cols, rows


def generate_board(inner_cols: int, inner_rows: int, square_mm: float, dpi: int, margin_mm: float) -> np.ndarray:
    squares_x = inner_cols + 1
    squares_y = inner_rows + 1

    pixels_per_mm = dpi / 25.4
    square_px = max(1, int(round(square_mm * pixels_per_mm)))
    margin_px = max(0, int(round(margin_mm * pixels_per_mm)))

    width = squares_x * square_px + 2 * margin_px
    height = squares_y * square_px + 2 * margin_px

    board = np.ones((height, width), dtype=np.uint8) * 255
    for y in range(squares_y):
        for x in range(squares_x):
            if (x + y) % 2 == 0:
                x0 = margin_px + x * square_px
                y0 = margin_px + y * square_px
                board[y0:y0 + square_px, x0:x0 + square_px] = 0
    return board


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a chessboard image for camera calibration")
    parser.add_argument('--size', default='8x6', help='Inner corners as COLSxROWS (default: 8x6)')
    parser.add_argument('--square-mm', type=float, default=25.0, help='Square size in mm (default: 25.0)')
    parser.add_argument('--dpi', type=int, default=300, help='Output DPI (default: 300)')
    parser.add_argument('--margin-mm', type=float, default=5.0, help='White margin in mm (default: 5.0)')
    parser.add_argument('--output', default='chessboard.png', help='Output PNG file path')
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        cols, rows = parse_size(args.size)
    except Exception as exc:
        print(f"invalid --size: {exc}", file=sys.stderr)
        return 2

    board = generate_board(cols, rows, args.square_mm, args.dpi, args.margin_mm)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    if not cv2.imwrite(str(output), board):
        print("failed to write output image", file=sys.stderr)
        return 1

    print(f"wrote {output} ({board.shape[1]}x{board.shape[0]} px, dpi={args.dpi})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
