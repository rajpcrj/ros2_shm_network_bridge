#!/usr/bin/env python3
"""
readme_to_pdf.py — render README.md to a clean, printable PDF.
Markdown -> styled HTML (python-markdown) -> PDF (headless Chrome).
Embedded PNG diagrams/graphs are resolved relative to the repo root, so they
appear in the PDF. Output: README.pdf in the repo root.

Run:  python3 docs/readme_to_pdf.py
"""
import os
import subprocess
import sys

import markdown

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MD = os.path.join(REPO, "README.md")
HTML = os.path.join(REPO, "README.rendered.html")
PDF = os.path.join(REPO, "README.pdf")

CSS = """
@page { size: A4; margin: 18mm 16mm; }
body { font-family: -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
       font-size: 10.5pt; line-height: 1.5; color: #1c1c1c; max-width: 100%; }
h1 { font-size: 22pt; border-bottom: 2px solid #1b9e77; padding-bottom: 6px; }
h2 { font-size: 16pt; border-bottom: 1px solid #ddd; padding-bottom: 4px; margin-top: 26px; }
h3 { font-size: 13pt; color: #2c3e50; margin-top: 18px; }
h4 { font-size: 11.5pt; color: #34495e; }
code { background: #f3f4f6; padding: 1px 4px; border-radius: 3px;
       font-family: "SFMono-Regular", Consolas, monospace; font-size: 9pt; }
pre { background: #f6f8fa; border: 1px solid #e1e4e8; border-radius: 6px;
      padding: 10px 12px; overflow-x: auto; page-break-inside: avoid; }
pre code { background: none; padding: 0; font-size: 8.5pt; line-height: 1.45; }
table { border-collapse: collapse; width: 100%; margin: 12px 0; font-size: 9pt;
        page-break-inside: avoid; }
th, td { border: 1px solid #d0d7de; padding: 5px 8px; text-align: left; vertical-align: top; }
th { background: #f0f3f5; }
img { max-width: 100%; height: auto; display: block; margin: 12px auto;
      page-break-inside: avoid; }
blockquote { border-left: 4px solid #1b9e77; background: #f3fbf8; margin: 12px 0;
             padding: 8px 14px; color: #2c3e50; }
a { color: #0b6cc4; text-decoration: none; }
h1, h2, h3 { page-break-after: avoid; }
"""


def main():
    with open(MD, encoding="utf-8") as f:
        text = f.read()
    body = markdown.markdown(
        text, extensions=["tables", "fenced_code", "toc", "sane_lists"])
    html = (f"<!doctype html><html><head><meta charset='utf-8'>"
            f"<style>{CSS}</style></head><body>{body}</body></html>")
    with open(HTML, "w", encoding="utf-8") as f:
        f.write(html)

    # find a chrome/chromium binary
    chrome = None
    for c in ("google-chrome", "chromium", "chromium-browser", "google-chrome-stable"):
        if subprocess.run(["which", c], capture_output=True).returncode == 0:
            chrome = c; break
    if not chrome:
        print("No Chrome/Chromium found; HTML written to", HTML); sys.exit(1)

    subprocess.run([
        chrome, "--headless", "--disable-gpu", "--no-sandbox",
        "--no-pdf-header-footer",
        f"--print-to-pdf={PDF}", f"file://{HTML}",
    ], check=True, capture_output=True)
    os.remove(HTML)
    print("wrote", PDF, "(%.0f KB)" % (os.path.getsize(PDF) / 1024))


if __name__ == "__main__":
    main()
