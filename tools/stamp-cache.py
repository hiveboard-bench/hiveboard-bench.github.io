#!/usr/bin/env python3
"""Cache-bust the assets vite copies verbatim.

Vite content-hashes its own bundles, but anything under public/ is copied as
is and referenced by a plain URL -- the favicon, the font stylesheet, the
simulator iframe.  Browsers and the Pages CDN then hold those indefinitely,
so a deploy appears not to have taken effect.  This stamps each such
reference with a short hash of the file it points at, so the URL changes
exactly when the content does.

Run after `npm run build`, against dist/.
"""
import hashlib
import re
import sys
from pathlib import Path

DIST = Path(__file__).resolve().parent.parent / "dist"

STAMP = [
    "./fonts/fonts.css",
    "./assets/favicon.svg",
    "./favicon.ico",
    "./assets/favicon.png",
    "./sim/hiveboard-sim.html",
]

ALSO = {"./sim/hiveboard-sim.html": ["sim/models", "sim/vendor"]}


def digest(path: Path, extra=()) -> str:

    sha = hashlib.sha256(path.read_bytes())
    for folder in extra:
        for child in sorted((DIST / folder).rglob("*")):
            if child.is_file():
                sha.update(child.name.encode())
                sha.update(hashlib.sha256(child.read_bytes()).digest())
    return sha.hexdigest()[:8]


def stamp_fonts():

    css = DIST / "fonts/fonts.css"
    if not css.exists():
        return {}

    text = css.read_text()
    stamps = {}
    for font in sorted((DIST / "fonts").glob("*.woff2")):
        stamps[font.name] = digest(font)
        text = re.sub(r"url\('%s(\?v=[0-9a-f]+)?'\)" % re.escape(font.name),
                      "url('%s?v=%s')" % (font.name, stamps[font.name]), text)
    css.write_text(text)
    print(f"  fonts.css: {len(stamps)} font url(s) versioned")
    return stamps


def main():

    page = DIST / "index.html"
    if not page.exists():
        sys.exit(f"no {page} -- run `npm run build` first")

    fonts = stamp_fonts()

    html = page.read_text()
    for name, tag in fonts.items():
        html = re.sub(r'"\./fonts/%s(\?v=[0-9a-f]+)?"' % re.escape(name),
                      '"./fonts/%s?v=%s"' % (name, tag), html)

    stamped = []
    for ref in STAMP:
        target = DIST / ref[2:]
        if not target.exists():
            print(f"  skip {ref} (not built)")
            continue

        pattern = re.escape(ref) + r'(\?v=[0-9a-f]+)?'
        new = f"{ref}?v={digest(target, ALSO.get(ref, ()))}"
        html, n = re.subn(f'"{pattern}"', f'"{new}"', html)
        if n:
            stamped.append(f"{ref} -> {new.rsplit('=', 1)[1]} ({n}x)")
        else:
            print(f"  warn {ref} not referenced in index.html")

    page.write_text(html)
    for line in stamped:
        print(f"  {line}")
    print(f"stamped {len(stamped)} reference(s)")


if __name__ == "__main__":
    main()
