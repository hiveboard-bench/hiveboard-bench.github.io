#!/usr/bin/env python3
import glob
import os
from PIL import Image

SRC = "assets-src"
OUT = "public/assets"

HERO = {"Hiveboard4": 500, "spot": 600, "anymal": 600, "hand": 560, "s010": 460}


def convert(path, out, width=None, box=None, **kw):

    im = Image.open(path)
    if im.mode != "RGBA":
        im = im.convert("RGBA")
    if width and im.width > width:
        im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
    if box:
        im.thumbnail((box, box), Image.LANCZOS)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    im.save(out, "WEBP", method=6, **kw)
    return os.path.getsize(path), os.path.getsize(out)


def main():

    before = after = 0
    jobs = []

    for name, w in HERO.items():
        jobs.append((f"{SRC}/{name}.png", f"{OUT}/{name}.webp", dict(width=w, quality=90)))

    jobs.append((f"{SRC}/hiveboard_preview.png", f"{OUT}/hiveboard_preview.webp",
                 dict(lossless=True)))

    for p in sorted(glob.glob(f"{SRC}/modules/*_setup.png")):
        jobs.append((p, f"{OUT}/modules/{os.path.basename(p)[:-4]}.webp",
                     dict(box=1200, quality=85)))

    for p in sorted(glob.glob(f"{SRC}/modules/*_3d.png")):
        jobs.append((p, f"{OUT}/modules/{os.path.basename(p)[:-4]}.webp",
                     dict(lossless=True)))

    for src, out, kw in jobs:
        if not os.path.exists(src):
            print(f"  skip (missing): {src}")
            continue
        a, b = convert(src, out, **kw)
        before += a
        after += b

    print(f"{len(jobs)} images: {before / 1048576:.2f} MB -> {after / 1024:.0f} KB "
          f"({100 - 100 * after // before}% smaller)")


if __name__ == "__main__":
    main()
