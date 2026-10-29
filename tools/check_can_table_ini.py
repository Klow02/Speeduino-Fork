#!/usr/bin/env python3
"""
check_can_table_ini.py - sanity checks of the CAN table (page 16) and gearbox (page 17) parts of reference/speeduino.ini.
Run from the repository root:  python3 tools/check_can_table_ini.py
(TunerStudio itself is not needed. It cannot catch everything TunerStudio would complain about, but
it finds the typical generator mistakes: duplicate names, overlapping bytes, dialogs that use
undefined fields.)

Checks
  1. every page-16 constant has a unique name (also unique in the whole [Constants] section),
  2. no two fields overlap in the 876 bytes (page 16) / 64 bytes (page 17) (bit fields may share a byte, but not the same bit),
  3. every field lies inside the page,
  4. every field/panel/subMenu name used by the CAN table dialogs is defined,
  5. nPages and all per-page lists have 17 entries; pageSize[16] = 876, pageSize[17] = 64,
  6. every gearbox default value ([ConstantsExtensions]) names a page-17 constant.
"""
import re
import sys
from pathlib import Path

ini = (Path(__file__).resolve().parent.parent / "reference" / "speeduino.ini").read_text().splitlines()
errors = []


def err(msg):
    errors.append(msg)


def section(name):
    start = ini.index("[%s]" % name)
    end = next((i for i in range(start + 1, len(ini)) if re.match(r"^\[\w+\]", ini[i])), len(ini))
    return ini[start + 1:end]


# ---- 1/2/3: constants -------------------------------------------------------------------------
consts = section("Constants")
names = {}
page = None
bytes_used = {}   # (byte) -> list of (name, bitmask)
FORK_PAGES = {16: 876, 17: 64}   # page number -> size in bytes
SIZES = {"U08": 1, "S08": 1, "U16": 2, "S16": 2}
for line in consts:
    m = re.match(r"^page\s*=\s*(\d+)", line)
    if m:
        page = int(m.group(1))
        continue
    m = re.match(r"^\s*(\w+)\s*=\s*(scalar|bits|array),\s*(\w+),\s*(\d+),\s*(.*)$", line)
    if not m:
        continue
    name, kind, typ, off, rest = m.group(1), m.group(2), m.group(3), int(m.group(4)), m.group(5)
    # (The upstream ini legitimately repeats names inside #if/#else branches, so only names of
    #  page 16 are checked.)
    if name in names and (page in FORK_PAGES or names[name] in FORK_PAGES):
        err("duplicate constant name %s (pages %s and %s)" % (name, names[name], page))
    names[name] = page
    if page not in FORK_PAGES:
        continue
    size = SIZES[typ]
    mask = 0xFF
    if kind == "bits":
        b = re.match(r"\[(\d+):(\d+)\]", rest)
        lo, hi = int(b.group(1)), int(b.group(2))
        mask = ((1 << (hi - lo + 1)) - 1) << lo
    if off + size > FORK_PAGES[page]:
        err("%s lies outside page %d" % (name, page))
    for k in range(size):
        bytes_used.setdefault((page, off + k), []).append((name, mask if size == 1 else 0xFF))
for byte, users in bytes_used.items():
    acc = 0
    for name, mask in users:
        if acc & mask:
            err("page %d byte %d: %s overlaps another field" % (byte[0], byte[1], name))
        acc |= mask
n16 = sum(1 for v in names.values() if v == 16)
n17 = sum(1 for v in names.values() if v == 17)
print("page 16 constants:", n16, " page 17 constants:", n17)
if n17 == 0:
    err("no page 17 constants")

# ---- 4: dialogs -------------------------------------------------------------------------------
ud = section("UserDefined")
dialogs = {m.group(1) for l in ud for m in [re.match(r"^\s*dialog\s*=\s*(\w+)", l)] if m}
for l in ud:
    m = re.match(r"^\s*field\s*=\s*\"[^\"]*\",\s*(\w+)", l)
    if m and re.match(r"(can(Ch|Msg|Sig)\d|gb[A-Z])", m.group(1)) and m.group(1) not in names:
        err("dialog uses undefined field " + m.group(1))
    m = re.match(r"^\s*panel\s*=\s*(\w+)", l)
    if m and re.match(r"(can(Msg|Sig)Row\d|gb[A-Z])", m.group(1)) and m.group(1) not in dialogs:
        err("dialog uses undefined panel " + m.group(1))
for l in section("Menu"):
    m = re.match(r"^\s*subMenu\s*=\s*((?:canTbl_|gbController)\w*)", l)
    if m and m.group(1) not in dialogs:
        err("menu uses undefined dialog " + m.group(1))

# ---- 5: page lists ----------------------------------------------------------------------------
txt = "\n".join(consts)
if not re.search(r"nPages\s*=\s*17\b", txt):
    err("nPages is not 17")
for key in ("pageSize", "pageIdentifier", "pageReadCommand", "pageValueWrite", "pageChunkWrite",
            "crc32CheckCommand", "burnCommand"):
    for m in re.finditer(r"^\s*%s\s*=\s*(.*)$" % key, txt, flags=re.M):
        count = len([i for i in m.group(1).split(",") if i.strip()])
        if count != 17:
            err("%s has %d entries instead of 17" % (key, count))
sizes = re.search(r"pageSize\s*=\s*(.*)", txt).group(1).split(",")
if int(sizes[15]) != 876:
    err("pageSize[16] is not 876")
if int(sizes[16]) != 64:
    err("pageSize[17] is not 64")

# ---- 6: gearbox defaults ----------------------------------------------------------------------
for l in section("ConstantsExtensions"):
    m = re.match(r"^\s*defaultValue\s*=\s*(gb\w+)\s*,", l)
    if m and names.get(m.group(1)) != 17:
        err("default value for unknown gearbox constant " + m.group(1))

if errors:
    print("\n".join("ERROR: " + e for e in errors))
    sys.exit(1)
print("ini check OK")
