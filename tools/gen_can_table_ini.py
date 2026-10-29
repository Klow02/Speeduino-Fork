#!/usr/bin/env python3
"""
gen_can_table_ini.py - writes the TunerStudio definition of page 16 (CAN message table)
into reference/speeduino.ini.

WHY A GENERATOR?
  The CAN table has 16 messages x 5 settings and 64 signals x 10 settings. In the .ini file every
  single setting needs its own line (TunerStudio has no "array of structures"), which would be
  ~650 hand-typed lines with byte offsets. A script cannot mis-count an offset.

WHAT IT DOES
  1. Page 15 -> 16 pages: updates nPages and the per-page lists (size, identifier, commands).
  2. Inserts the "page = 16" constants after page 15.
  3. Inserts the dialogs ([UserDefined]) and one menu entry ([Menu]).
  Every inserted block is wrapped in  "; >>> CAN TABLE ... <<<"  marker lines, so running the script
  again replaces the old blocks instead of adding them twice.

USAGE   python3 tools/gen_can_table_ini.py            (from the repository root)

THE BYTE LAYOUT MUST MATCH  speeduino/src/controllers/canTable/canTable.h  (struct canTableConfig_t).
The firmware checks the offsets used here with static_asserts in config_pages.h, so a change of the
struct that is not mirrored here will not compile.
"""
import re
import sys
from pathlib import Path

INI = Path(__file__).resolve().parent.parent / "reference" / "speeduino.ini"

# ---- layout (bytes), mirrors canTable.h --------------------------------------------------------
N_CHANNELS, CH_SIZE, CH_BASE = 2, 6, 0
N_MESSAGES, MSG_SIZE, MSG_BASE = 16, 6, 12
N_SIGNALS, SIG_SIZE, SIG_BASE = 64, 12, 108
PAGE_NUMBER, PAGE_SIZE = 16, 876
assert MSG_BASE == CH_BASE + N_CHANNELS * CH_SIZE
assert SIG_BASE == MSG_BASE + N_MESSAGES * MSG_SIZE
assert PAGE_SIZE == SIG_BASE + N_SIGNALS * SIG_SIZE

# Hardware name of the two usable internal CAN controllers of the Teensy 4.1 (channel index 0 / 1).
CHANNEL_NAMES = ["CAN1", "CAN3"]

# Order MUST match enum canTableSource_t in canTable.h.
SOURCES = ["None (sends 0)", "RPM", "MAP kPa", "TPS %", "Coolant C", "Intake air C",
           "Battery 0.1V", "Vehicle speed km/h", "Gear", "Oil pressure kPa", "Fuel pressure kPa",
           "Fuel temp C", "Lambda x1000",
           "User 1", "User 2", "User 3", "User 4", "User 5", "User 6", "User 7", "User 8"]


def quote(items):
    return ", ".join('"%s"' % i for i in items)


def constants():
    """The [Constants] lines of page 16."""
    L = []
    a = L.append
    a("page = %d" % PAGE_NUMBER)
    a(";CAN message table (fork addition). Layout = struct canTableConfig_t, see canTable.h.")
    a(";Channel settings: 2 x 6 bytes")
    for c, name in enumerate(CHANNEL_NAMES):
        b = CH_BASE + c * CH_SIZE
        p = "canCh%d_" % (c + 1 if c == 0 else 3)
        a('      %senabled  = bits,   U08,  %3d, [0:0], "Off", "On"' % (p, b))
        a('      %sTermPin  = scalar, U08,  %3d, "pin", 1.0, 0.0, 0, 255, 0' % (p, b + 1))
        a('      %sBitrate  = scalar, U16,  %3d, "kbit/s", 1.0, 0.0, 10, 1000, 0' % (p, b + 2))
    a(";Messages: 16 x 6 bytes")
    for m in range(N_MESSAGES):
        b = MSG_BASE + m * MSG_SIZE
        p = "canMsg%d_" % (m + 1)
        a('      %sen    = bits,   U08,  %3d, [0:0], "Off", "On"' % (p, b + 4))
        a('      %sdir   = bits,   U08,  %3d, [1:1], "Send (out)", "Receive (in)"' % (p, b + 4))
        a('      %sch    = bits,   U08,  %3d, [2:2], "%s", "%s"' % (p, b + 4, CHANNEL_NAMES[0], CHANNEL_NAMES[1]))
        a('      %sid    = scalar, U16,  %3d, "ID",  1.0, 0.0, 0, 2047, 0' % (p, b))
        a('      %sint   = scalar, U16,  %3d, "ms",  1.0, 0.0, 0, 65535, 0' % (p, b + 2))
    a(";Signals: 64 x 12 bytes")
    src = quote(SOURCES + ["INVALID"] * (32 - len(SOURCES)))
    for s in range(N_SIGNALS):
        b = SIG_BASE + s * SIG_SIZE
        p = "canSig%d_" % (s + 1)
        a('      %sen    = bits,   U08,  %3d, [0:0], "Off", "On"' % (p, b + 4))
        a('      %ssign  = bits,   U08,  %3d, [1:1], "Unsigned", "Signed"' % (p, b + 4))
        a('      %sbig   = bits,   U08,  %3d, [2:2], "Little endian (Intel)", "Big endian (Motorola)"' % (p, b + 4))
        a('      %smsg   = scalar, U08,  %3d, "msg", 1.0, 1.0, 1, 16, 0' % (p, b))   # stored 0..15, shown 1..16
        a('      %ssrc   = bits,   U08,  %3d, [0:4], %s' % (p, b + 1, src))
        a('      %sstart = scalar, U08,  %3d, "bit", 1.0, 0.0, 0, 63, 0' % (p, b + 2))
        a('      %slen   = scalar, U08,  %3d, "bits", 1.0, 0.0, 1, 32, 0' % (p, b + 3))
        a('      %smul   = scalar, S16,  %3d, "x", 1.0, 0.0, -32768, 32767, 0' % (p, b + 6))
        a('      %sdiv   = scalar, S16,  %3d, "/", 1.0, 0.0, 1, 32767, 0' % (p, b + 8))
        a('      %soff   = scalar, S16,  %3d, "+", 1.0, 0.0, -32768, 32767, 0' % (p, b + 10))
    return L


def dialogs():
    """The [UserDefined] dialogs."""
    L = []
    a = L.append
    note = [
        'field = "wire value = round( source * multiplier / divisor ) + offset"',
        'field = "Bit 0 = lowest bit of byte 0. Signals of one message must not overlap."',
    ]
    # channel dialog
    for c, name in enumerate(CHANNEL_NAMES):
        p = "canCh%d_" % (1 if c == 0 else 3)
        a('    dialog = canTbl_%s, "%s", yAxis' % (name.lower(), name))
        a('        field = "Channel active",                 %senabled' % p)
        a('        field = "Bitrate",                        %sBitrate, { %senabled }' % (p, p))
        a('        field = "Termination pin (255 = none)",   %sTermPin, { %senabled }' % (p, p))
    a('    dialog = canTbl_channels, "CAN table: channels", yAxis')
    a('        field = "Needs an external CAN transceiver. CAN1 shares the bus speed with the normal Speeduino CAN."')
    for name in CHANNEL_NAMES:
        a('        panel = canTbl_%s' % name.lower())
    # message rows
    for m in range(N_MESSAGES):
        p = "canMsg%d_" % (m + 1)
        a('    dialog = canMsgRow%d, "", xAxis' % (m + 1))
        a('        field = "%2d  Active",   %sen' % (m + 1, p))
        a('        field = "ID",            %sid,  { %sen }' % (p, p))
        a('        field = "Direction",     %sdir, { %sen }' % (p, p))
        a('        field = "Channel",       %sch,  { %sen }' % (p, p))
        a('        field = "Every (ms)",    %sint, { %sen }' % (p, p))
    a('    dialog = canTbl_messages, "CAN table: messages", yAxis')
    a('        field = "ID is entered in DECIMAL (0x011 = 17, 0x033 = 51, 0x055 = 85, 0x066 = 102). Empty table = nothing is sent."')
    for m in range(N_MESSAGES):
        a('        panel = canMsgRow%d' % (m + 1))
    # signal rows, 16 per dialog
    for s in range(N_SIGNALS):
        p = "canSig%d_" % (s + 1)
        a('    dialog = canSigRow%d, "", xAxis' % (s + 1))
        a('        field = "%2d  Active",   %sen' % (s + 1, p))
        a('        field = "Message",       %smsg,   { %sen }' % (p, p))
        a('        field = "Value",         %ssrc,   { %sen }' % (p, p))
        a('        field = "Start bit",     %sstart, { %sen }' % (p, p))
        a('        field = "Length",        %slen,   { %sen }' % (p, p))
        a('        field = "Sign",          %ssign,  { %sen }' % (p, p))
        a('        field = "Byte order",    %sbig,   { %sen }' % (p, p))
        a('        field = "x",             %smul,   { %sen }' % (p, p))
        a('        field = "/",             %sdiv,   { %sen }' % (p, p))
        a('        field = "+",             %soff,   { %sen }' % (p, p))
    for g in range(N_SIGNALS // 16):
        a('    dialog = canTbl_signals%d, "CAN table: signals %d-%d", yAxis' % (g + 1, g * 16 + 1, g * 16 + 16))
        for n in note:
            a('        ' + n)
        for s in range(g * 16, g * 16 + 16):
            a('        panel = canSigRow%d' % (s + 1))
    return L


def menu():
    L = []
    L.append('        subMenu = std_separator')
    L.append('        subMenu = canTbl_channels,   "CAN table: channels"')
    L.append('        subMenu = canTbl_messages,   "CAN table: messages"')
    for g in range(N_SIGNALS // 16):
        L.append('        subMenu = canTbl_signals%d,   "CAN table: signals %d-%d"' % (g + 1, g * 16 + 1, g * 16 + 16))
    return L


def block(name, lines):
    return ["; >>> CAN TABLE %s (generated by tools/gen_can_table_ini.py, do not edit by hand) <<<" % name] + lines + \
           ["; >>> END CAN TABLE %s <<<" % name]


def strip_blocks(text):
    return re.sub(r"; >>> CAN TABLE (\w+) .*?; >>> END CAN TABLE \1 <<<\n", "", text, flags=re.S)


def fix_page_lists(text):
    """nPages and the per-page lists get a 16th entry. A list that already has 16 entries is left alone,
    so running the script twice does not add a 17th."""
    def add_item(key, new_item=None):
        # new_item None = repeat the last item of the list (all pages use the same command)
        def repl(m):
            items = [i.strip() for i in m.group(2).split(",")]
            if len(items) != PAGE_NUMBER - 1:
                return m.group(0)
            return m.group(0) + ", " + (new_item if new_item is not None else items[-1])
        return re.subn(r"^(\s*%s\s*=\s*)(.*)$" % key, repl, text, flags=re.M)

    nonlocal_text = re.sub(r"nPages\s*=\s*\d+", "nPages              = %d" % PAGE_NUMBER, text, count=1)
    text = nonlocal_text
    for key, item in (("pageSize", str(PAGE_SIZE)), ("pageIdentifier", r'"\$tsCanId\x%02X"' % PAGE_NUMBER),
                      ("pageReadCommand", None), ("pageValueWrite", None), ("pageChunkWrite", None),
                      ("crc32CheckCommand", None), ("burnCommand", None)):
        text, n = add_item(key, item)
        if n == 0:
            sys.exit("ini key not found: " + key)
    return text


def main():
    text = INI.read_text()
    text = strip_blocks(text)
    text = fix_page_lists(text)

    # constants: after the last line of page 15 (before the ';-----' line that precedes [EventTriggers])
    marker = "\n;-------------------------------------------------------------------------------\n\n[EventTriggers]"
    if marker not in text:
        sys.exit("cannot find the end of [Constants]")
    text = text.replace(marker, "\n".join(block("PAGE", constants())) + "\n" + marker, 1)

    # dialogs: directly after the [UserDefined] line
    text = text.replace("\n[UserDefined]\n", "\n[UserDefined]\n" + "\n".join(block("DIALOGS", dialogs())) + "\n", 1)

    # menu: directly after the CAN broadcast entry
    anchor = '        subMenu = CanBcast,             "CAN Broadcasting menu", { CANisAvailable }\n'
    if anchor not in text:
        sys.exit("cannot find the CAN menu entry")
    text = text.replace(anchor, anchor + "\n".join(block("MENU", menu())) + "\n", 1)

    INI.write_text(text)
    print("written", INI)


if __name__ == "__main__":
    main()
