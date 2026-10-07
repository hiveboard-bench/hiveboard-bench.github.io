#!/usr/bin/env python3
import os
import re
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from concurrent.futures import ThreadPoolExecutor

from PIL import Image, ImageStat

OUT = "public/assets/posters"
INDEX = "index.html"
HEAD_BYTES = 2_500_000
TIMESTAMPS = ("0.5", "1.5", "3.0", "5.0")
WIDTH = 800
HEIGHT = 450
QUALITY = 78
UA = {"User-Agent": "Mozilla/5.0"}

FOCUS: dict[str, float] = {
    "macao_big_valve": 0.64,
    "macao_box": 0.70,
    "macao_bulb_lamp": 0.68,
    "macao_button": 0.70,
    "macao_circuit_breaker": 0.69,
    "macao_key": 0.69,
    "macao_peg_and_hole": 0.69,
    "macao_small_valve": 0.67,
    "macao_spring": 0.70,
    "macao_torque_valve": 0.67,
    "s010_m30_exp5": 0.67,
    "s010_m8_exp5": 0.67,
}
TIME: dict[str, str] = {}


def ffmpeg_bin():

    local = "node_modules/ffmpeg-static/ffmpeg"
    if os.path.exists(local):
        return local
    found = shutil.which("ffmpeg")
    if not found:
        sys.exit("ffmpeg not found. Run: npm i ffmpeg-static")
    return found


FFMPEG = ffmpeg_bin()


def video_urls():

    html = open(INDEX, encoding="utf-8").read()
    urls = sorted(set(re.findall(r'src="(https://[^"#]+\.mp4)(?:#[^"]*)?"', html)))
    if not urls:
        sys.exit("no video URLs found in " + INDEX)
    return urls


def fetch_head(url, nbytes):

    req = urllib.request.Request(url, headers={**UA, "Range": f"bytes=0-{nbytes - 1}"})
    return urllib.request.urlopen(req).read()


def score(path):

    im = Image.open(path).convert("L")
    st = ImageStat.Stat(im)
    mean, sd = st.mean[0], st.stddev[0]
    return (mean >= 25 and sd >= 12), sd


def strip_letterbox(im):

    mask = im.convert("L").point(lambda v: 255 if v > 18 else 0)
    box = mask.getbbox()
    if not box:
        return im
    if box[2] - box[0] < im.width * 0.2 or box[3] - box[1] < im.height * 0.2:
        return im
    return im.crop(box)


def crop_to_ratio(im, target, focus):

    if im.width / im.height > target:
        w = round(im.height * target)
        left = min(max(round(im.width * focus - w / 2), 0), im.width - w)
        return im.crop((left, 0, left + w, im.height))
    h = round(im.width / target)
    top = min(max(round(im.height * focus - h / 2), 0), im.height - h)
    return im.crop((0, top, im.width, top + h))


def poster_for(url):

    name = url.rsplit("/", 1)[-1][:-4]
    out = f"{OUT}/{name}.webp"
    tmpdir = tempfile.mkdtemp()
    try:
        clip = os.path.join(tmpdir, "head.mp4")
        with open(clip, "wb") as fh:
            fh.write(fetch_head(url, HEAD_BYTES))

        stamps = (TIME[name],) + TIMESTAMPS if name in TIME else TIMESTAMPS
        best, best_sd = None, -1.0
        for ts in stamps:
            frame = os.path.join(tmpdir, f"f{ts}.png")
            subprocess.run(
                [FFMPEG, "-nostdin", "-loglevel", "error", "-ss", ts, "-i", clip,
                 "-frames:v", "1", "-y", frame],
                capture_output=True,
            )
            if not os.path.exists(frame):
                continue
            good, sd = score(frame)
            if sd > best_sd:
                best, best_sd = frame, sd
            if good:
                best = frame
                break

        if best is None:
            return name, None, None, "no frame could be decoded"

        im = Image.open(best).convert("RGB")
        im = strip_letterbox(im)
        target = WIDTH / HEIGHT
        focus = FOCUS.get(name, 0.5)
        im = crop_to_ratio(im, target, focus)
        im = im.resize((WIDTH, HEIGHT), Image.LANCZOS)
        os.makedirs(OUT, exist_ok=True)
        im.save(out, "WEBP", quality=QUALITY, method=6)
        return name, os.path.getsize(out), round(focus, 3), None
    except Exception as exc:
        return name, None, None, str(exc)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


def main():

    urls = video_urls()
    print(f"generating {len(urls)} posters into {OUT}/ ...")
    total, failed, placed = 0, [], []
    with ThreadPoolExecutor(max_workers=6) as pool:
        for name, size, focus, err in pool.map(poster_for, urls):
            if err:
                failed.append((name, err))
                print(f"  FAIL {name}: {err}")
            else:
                total += size
                placed.append((name, focus))
    ok = len(urls) - len(failed)
    print(f"{ok}/{len(urls)} posters, {total / 1024:.0f} KB total "
          f"({total / ok / 1024:.0f} KB average)" if ok else "nothing generated")
    off = [(n, f) for n, f in placed if abs(f - 0.5) > 0.12]
    if off:
        print("\nauto-placed crop away from centre (set FOCUS to override):")
        for n, f in sorted(off): print(f"    {n:28} focus={f}")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
