r"""
Export the 2D map stored in an RTAB-Map database as a map_server image (.pgm + .yaml).

    ros2 run stretch4_rtabmap export_map ~/stretch_maps/<my_map>.db
    ros2 run stretch4_rtabmap export_map ~/stretch_maps/<my_map>.db \
        -o $HELLO_FLEET_PATH/maps/<my_map>/map

Writes <output>.pgm and <output>.yaml (default output: the database path without .db).
RTAB-Map writes this map into the database when it shuts down cleanly (Ctrl-C), so it is the map
as of the end of the last run. To grab the map while RTAB-Map is running instead:
    ros2 run nav2_map_server map_saver_cli -f <output>

Read-only; safe on a database RTAB-Map has open.
"""

import argparse
import os
import sqlite3
import struct
import sys
import zlib

import numpy as np

# map_server's trinary image values, as nav2's map_saver writes them.
FREE, OCCUPIED, UNKNOWN = 254, 0, 205


def load_map(db_path):
    """Return (grid, x_min, y_min, resolution, saved_at); grid is int8 -1/0/100, row 0 = y_min."""
    with sqlite3.connect(f'file:{db_path}?mode=ro', uri=True) as db:
        row = db.execute('SELECT opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution, '
                         'time_enter FROM Admin').fetchone()
    blob, x_min, y_min, resolution, saved_at = row
    if not blob:
        return None
    # rtabmap::compressData2: zlib data, then rows, cols and type as three int32.
    rows, cols, _ = struct.unpack('<3i', blob[-12:])
    grid = np.frombuffer(zlib.decompress(blob[:-12]), dtype=np.int8).reshape(rows, cols)
    return grid, x_min, y_min, resolution, saved_at


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('database', help='RTAB-Map database (.db)')
    parser.add_argument('-o', '--output',
                        help='output path without extension (default: database path minus .db)')
    parser.add_argument('--margin', type=float, default=1.0,
                        help='metres of unknown kept around the mapped area (default 1.0)')
    args = parser.parse_args()

    db_path = os.path.expanduser(args.database)
    if not os.path.isfile(db_path):
        sys.exit(f'no such file: {db_path}')
    loaded = load_map(db_path)
    if loaded is None:
        sys.exit(f'{db_path} holds no 2D map. RTAB-Map saves it when it shuts down cleanly: run '
                 'the navigation launch on this database once and stop it with Ctrl-C.')
    grid, x_min, y_min, resolution, saved_at = loaded

    # Crop to the mapped area plus a margin; the origin moves with the crop.
    known_rows, known_cols = np.nonzero(grid >= 0)
    if known_rows.size == 0:
        sys.exit(f'{db_path}: the stored map is empty.')
    pad = int(round(args.margin / resolution))
    r0, r1 = max(known_rows.min() - pad, 0), min(known_rows.max() + pad + 1, grid.shape[0])
    c0, c1 = max(known_cols.min() - pad, 0), min(known_cols.max() + pad + 1, grid.shape[1])
    grid = grid[r0:r1, c0:c1]
    origin = (x_min + c0 * resolution, y_min + r0 * resolution)

    image = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    image[grid == 0] = FREE
    image[grid > 0] = OCCUPIED
    image = np.flipud(image)  # image row 0 is the top of the map (largest y)

    output = os.path.expanduser(args.output) if args.output else os.path.splitext(db_path)[0]
    out_dir = os.path.dirname(os.path.abspath(output))
    os.makedirs(out_dir, exist_ok=True)
    height, width = image.shape
    with open(output + '.pgm', 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(image.tobytes())
    with open(output + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(output)}.pgm\n'
                'mode: trinary\n'
                f'resolution: {resolution:.4f}\n'
                f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.25\n')

    print(f'{output}.yaml: {width}x{height} cells at {resolution:.3f} m, '
          f'origin ({origin[0]:.2f}, {origin[1]:.2f}), saved by RTAB-Map {saved_at} UTC')


if __name__ == '__main__':
    main()
