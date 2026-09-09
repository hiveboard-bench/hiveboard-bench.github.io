#!/usr/bin/env python3
"""Fill in the sitemap's <lastmod> at build time.

public/sitemap.xml is copied verbatim by vite, so its <lastmod> would be
whatever was hand-written there -- stale the moment the page changes.  This
rewrites it in dist/ with the date of the last commit touching the site, which
is the only field crawlers actually use to schedule a recrawl (changefreq and
priority are ignored, so the sitemap does not carry them).

Run after `npm run build`, against dist/.
"""
import re
import subprocess
import sys
from datetime import date, timezone, datetime
from pathlib import Path

SITEMAP = Path(__file__).resolve().parent.parent / "dist" / "sitemap.xml"


def last_modified() -> str:
    try:
        stamp = subprocess.run(
            ["git", "log", "-1", "--format=%cI"],
            cwd=SITEMAP.parent.parent,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        return datetime.fromisoformat(stamp).astimezone(timezone.utc).date().isoformat()
    except (subprocess.CalledProcessError, ValueError, FileNotFoundError):
        return date.today().isoformat()


def main() -> int:
    if not SITEMAP.exists():
        print("stamp-sitemap: dist/sitemap.xml missing -- run the build first", file=sys.stderr)
        return 1

    when = last_modified()
    text = SITEMAP.read_text()
    stamped, count = re.subn(r"<lastmod>[^<]*</lastmod>", f"<lastmod>{when}</lastmod>", text)
    if not count:
        print("stamp-sitemap: no <lastmod> to stamp", file=sys.stderr)
        return 1

    SITEMAP.write_text(stamped)
    print(f"stamp-sitemap: lastmod {when} ({count} url{'s' if count > 1 else ''})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
