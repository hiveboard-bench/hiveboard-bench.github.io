#!/usr/bin/env python3
import glob
import gzip
import os

MODELS = "public/models"


def main():

    raw = comp = 0
    count = 0
    for path in sorted(glob.glob(f"{MODELS}/**/*.stl", recursive=True)):
        data = open(path, "rb").read()
        packed = gzip.compress(data, 9)
        gz_path = path + ".gz"
        if not os.path.exists(gz_path) or gzip.decompress(open(gz_path, "rb").read()) != data:
            open(gz_path, "wb").write(packed)
        raw += len(data)
        comp += len(packed)
        count += 1

    stale = [p for p in glob.glob(f"{MODELS}/**/*.stl.gz", recursive=True)
             if not os.path.exists(p[:-3])]
    for p in stale:
        os.remove(p)
        print(f"  removed orphan: {p}")

    print(f"{count} meshes: {raw / 1048576:.2f} MB -> {comp / 1048576:.2f} MB "
          f"({100 - 100 * comp // raw}% smaller over the wire)")


if __name__ == "__main__":
    main()
