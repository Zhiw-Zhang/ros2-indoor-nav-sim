#!/usr/bin/env python3
"""把 xwd 转成 png（只支持 ZPixmap / 16、24 或 32 bpp 的真彩屏）。

    ros2 run my_robot_description xwd2png.py in.xwd out.png

配合：
    export DISPLAY=:0
    xwininfo -root -tree | grep -i rviz     # 找窗口 id
    xwd -id 0x600106 -out /tmp/shot.xwd

★ 坑：X 服务器的行距经常按 4 字节对齐（bpl == w*4），**即使头里写的是
  bpp=24**。所以下面按行距判断真实的每像素字节数，而不是信 bpp ——
  照 bpp=24 解会 reshape 失败。
"""
import argparse
import struct
import sys
import numpy as np
from PIL import Image


def convert(src, dst):
    with open(src, 'rb') as f:
        data = f.read()
    fld = struct.unpack('>25I', data[:100])
    (header_size, version, fmt, depth, w, h, xoff, byte_order, bitmap_unit,
     bit_order, pad, bpp, bpl, vclass, rmask, gmask, bmask, bits_rgb,
     cmap_entries, ncolors) = fld[:20]
    print(f'version={version} fmt={fmt} depth={depth} {w}x{h} bpp={bpp} bpl={bpl} '
          f'byte_order={"LSB" if byte_order == 0 else "MSB"} ncolors={ncolors}')

    off = header_size
    raw = data[off:off + bpl * h]

    # X 服务器的行距经常是 4 字节对齐的（bpl == w*4），即使头里写 bpp=24。
    # 所以按行距来判断真实的每像素字节数，而不是信 bpp。
    if bpl == w * 4 or bpp == 32:
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, bpl // 4, 4)[:, :w, :]
        rgb = arr[:, :, [2, 1, 0]]
    elif bpl == w * 3 or bpp == 24:
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, bpl // 3, 3)[:, :w, :]
        rgb = arr[:, :, [2, 1, 0]]
    elif bpp == 16:
        arr = np.frombuffer(raw, dtype='<u2').reshape(h, bpl // 2)[:, :w]
        r = ((arr >> 11) & 0x1F) * 255 // 31
        g = ((arr >> 5) & 0x3F) * 255 // 63
        b = (arr & 0x1F) * 255 // 31
        rgb = np.stack([r, g, b], -1).astype(np.uint8)
    else:
        raise SystemExit(f'不支持的 bpp={bpp}')

    Image.fromarray(rgb.astype(np.uint8)).save(dst)
    print(f'wrote {dst} ({w}, {h})')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', help='输入 .xwd（xwd -id <wid> -out 出来的）')
    ap.add_argument('dst', help='输出 .png')
    a = ap.parse_args()
    convert(a.src, a.dst)
