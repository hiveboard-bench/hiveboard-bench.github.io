#!/usr/bin/env python3
import io
import os
import re
import urllib.request

from fontTools.subset import Options, Subsetter
from fontTools.ttLib import TTFont

OUT = "public/fonts"
FA = "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/"
AC = "https://cdn.jsdelivr.net/gh/jpswalsh/academicons@1/"
GF = ("https://fonts.googleapis.com/css2"
      "?family=Poppins:wght@300;400;500;600;700&display=swap")
UA = {"User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 "
                    "(KHTML, like Gecko) Chrome/120.0 Safari/537.36"}

USED = {
    "solid": ["play-circle", "play", "chevron-right", "chevron-left", "cube",
              "network-wired", "microchip", "hand-pointer", "file-pdf",
              "check", "angle-down", "clipboard-check", "clipboard-list", "book-open"],
    "regular": ["copy"],
    "brands": ["python", "github"],
}
FILES = {"solid": "fa-solid-900", "regular": "fa-regular-400",
         "brands": "fa-brands-400"}
SUBSETS = ("latin", "latin-ext")


def get(url):

    return urllib.request.urlopen(urllib.request.Request(url, headers=UA)).read()


def subset_font(ttf_bytes, codepoints, out_path):

    font = TTFont(io.BytesIO(ttf_bytes))
    opts = Options()
    opts.flavor = "woff2"
    opts.layout_features = []
    opts.notdef_outline = False
    opts.desubroutinize = True
    ss = Subsetter(options=opts)
    ss.populate(unicodes=codepoints)
    ss.subset(font)
    font.flavor = "woff2"
    font.save(out_path)
    return os.path.getsize(out_path)


def main():

    os.makedirs(OUT, exist_ok=True)
    n_glyphs = sum(len(v) for v in USED.values()) + 1
    lines = [
        "/* Self-hosted fonts for HiveBoard.",
        "   Poppins: latin + latin-ext subsets, served from this origin instead of Google Fonts.",
        f"   Icons:   Font Awesome 6.4.0 + Academicons, subset to the {n_glyphs} glyphs this site uses.",
        "   Generated file - regenerate with tools/build-fonts.py if the icon set changes. */",
        "",
    ]

    css = get(GF).decode()
    faces = re.findall(r"/\*\s*([\w-]+)\s*\*/\s*@font-face\s*\{(.*?)\}", css, re.S)
    for name, body in faces:
        if name not in SUBSETS:
            continue
        weight = re.search(r"font-weight:\s*(\d+)", body).group(1)
        rng = re.search(r"unicode-range:\s*([^;]+);", body).group(1).strip()
        url = re.search(r"url\(([^)]+)\)", body).group(1)
        fname = f"poppins-{weight}-{name}.woff2"
        open(f"{OUT}/{fname}", "wb").write(get(url))
        lines.append(
            f"@font-face{{font-family:'Poppins';font-style:normal;font-weight:{weight};"
            f"font-display:swap;src:url('{fname}') format('woff2');unicode-range:{rng}}}")
    lines.append("")

    fa_css = get(FA + "css/all.min.css").decode()

    def cp(icon):

        m = re.search(r'\.fa-%s:{1,2}before\s*\{\s*content:\s*"\\([0-9a-f]+)"'
                      % re.escape(icon), fa_css)
        if not m:
            raise SystemExit(f"no codepoint for fa-{icon} in Font Awesome {FA}")
        return int(m.group(1), 16)

    glyphs, total = {}, 0
    for style, names in USED.items():
        for n in names:
            glyphs[n] = ("fa-", cp(n))
        total += subset_font(get(f"{FA}webfonts/{FILES[style]}.ttf"),
                             [cp(n) for n in names], f"{OUT}/{FILES[style]}.woff2")

    ac_css = get(AC + "css/academicons.min.css").decode()
    acp = int(re.search(r'\.ai-arxiv:{1,2}before\s*\{\s*content:\s*"\\([0-9a-f]+)"',
                        ac_css).group(1), 16)
    glyphs["arxiv"] = ("ai-", acp)
    total += subset_font(get(AC + "fonts/academicons.ttf"), [acp],
                         f"{OUT}/academicons.woff2")

    base = re.search(r"\{(-moz-osx-font-smoothing:grayscale;[^}]*text-rendering:auto)\}",
                     fa_css).group(1)
    lines += [
        "@font-face{font-family:'Font Awesome 6 Free';font-style:normal;font-weight:900;"
        "font-display:block;src:url('fa-solid-900.woff2') format('woff2')}",
        "@font-face{font-family:'Font Awesome 6 Free';font-style:normal;font-weight:400;"
        "font-display:block;src:url('fa-regular-400.woff2') format('woff2')}",
        "@font-face{font-family:'Font Awesome 6 Brands';font-style:normal;font-weight:400;"
        "font-display:block;src:url('fa-brands-400.woff2') format('woff2')}",
        "@font-face{font-family:'Academicons';font-style:normal;font-weight:400;"
        "font-display:block;src:url('academicons.woff2') format('woff2')}",
        "",
        f".fa,.fas,.far,.fab,.fa-solid,.fa-regular,.fa-brands{{{base}}}",
        ".fas,.fa-solid{font-family:'Font Awesome 6 Free';font-weight:900}",
        ".far,.fa-regular{font-family:'Font Awesome 6 Free';font-weight:400}",
        ".fab,.fa-brands{font-family:'Font Awesome 6 Brands';font-weight:400}",
        ".ai{font-family:'Academicons';font-weight:400;-moz-osx-font-smoothing:grayscale;"
        "-webkit-font-smoothing:antialiased;display:inline-block;font-style:normal;"
        "font-variant:normal;text-rendering:auto;line-height:1}",
        ".fa-lg{%s}" % re.search(r"\.fa-lg\{([^}]*)\}", fa_css).group(1),
        ".fa-3x{%s}" % re.search(r"\.fa-3x\{([^}]*)\}", fa_css).group(1),
        "",
    ]
    for name, (prefix, code) in sorted(glyphs.items()):
        lines.append('.%s%s:before{content:"\\%x"}' % (prefix, name, code))

    open(f"{OUT}/fonts.css", "w").write("\n".join(lines) + "\n")
    print(f"{len(glyphs)} icon glyphs -> {total} bytes of webfont, plus self-hosted Poppins")
    print(f"wrote {OUT}/fonts.css")


if __name__ == "__main__":
    main()
