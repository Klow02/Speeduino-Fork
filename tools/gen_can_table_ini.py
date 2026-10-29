#!/usr/bin/env python3
"""
gen_can_table_ini.py - writes the TunerStudio definition of page 16 (CAN message table) and
page 17 (paddle shift gearbox settings) into reference/speeduino.ini.

WHY A GENERATOR?
  The CAN table has 16 messages x 5 settings and 64 signals x 10 settings. In the .ini file every
  single setting needs its own line (TunerStudio has no "array of structures"), which would be
  ~650 hand-typed lines with byte offsets. A script cannot mis-count an offset.

WHAT IT DOES
  1. Page 15 -> 17 pages: updates nPages and the per-page lists (size, identifier, commands).
  2. Inserts the "page = 16" and "page = 17" constants after page 15.
  3. Inserts the dialogs ([UserDefined]), the menu entries ([Menu]) and default values
     ([ConstantsExtensions]) for the gearbox page.
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
GB_PAGE_NUMBER, GB_PAGE_SIZE = 17, 64      # gearbox page, mirrors gearboxSettings.h
N_PAGES = 17
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


# Gearbox page: (name, kind, byte, spec). Mirrors struct gearboxSettingsPage_t in gearboxSettings.h.
# kind "bit": spec = (byte, bit, text0, text1); kind "num": spec = (type, units, min, max)
GB_BITS = [
    ("gbEnabled",      0, 0, "Off", "On"),
    ("gbDownToNeutral", 0, 1, "No", "Yes"),
    ("gbCutUp",        0, 2, "No", "Yes"),
    ("gbCutDown",      0, 3, "No", "Yes"),
    ("gbCutIgn",       0, 4, "No", "Yes"),
    ("gbCutFuel",      0, 5, "No", "Yes"),
    ("gbFeedback",     0, 6, "Fixed times", "Position feedback"),
    ("gbPaddleLow",    1, 0, "Active high (to +V)", "Active low (to ground)"),
    ("gbNeutralLow",   1, 1, "Active high (to +V)", "Active low (to ground)"),
]
GB_NUMS = [
    ("gbMaxGear",   "U08",  2, "gear", 1, 6),
    ("gbPinUp",     "U08",  3, "pin", 0, 255),
    ("gbPinDown",   "U08",  4, "pin", 0, 255),
    ("gbPinNeutral", "U08", 5, "pin", 0, 255),
    ("gbCutMinRpm", "U16",  6, "rpm", 0, 20000),
    ("gbMaxCutMs",  "U16",  8, "ms", 0, 1000),
    ("gbDownMaxRpm", "U16", 10, "rpm", 0, 20000),
    ("gbUpTravel",  "U16", 12, "permille", 0, 1000),
    ("gbDownTravel", "U16", 14, "permille", 0, 1000),
    ("gbPosTol",    "U16", 16, "permille", 0, 1000),
    ("gbDebounce",  "U16", 18, "ms", 0, 1000),
    ("gbCutLead",   "U16", 20, "ms", 0, 1000),
    ("gbMoveMs",    "U16", 22, "ms", 0, 5000),
    ("gbHoldMs",    "U16", 24, "ms", 0, 5000),
    ("gbReturnMs",  "U16", 26, "ms", 0, 5000),
    ("gbVerifyMs",  "U16", 28, "ms", 0, 5000),
    ("gbMinInterval", "U16", 30, "ms", 0, 10000),
    ("gbMaxShiftMs", "U16", 32, "ms", 0, 10000),
    ("gbFaultLock", "U16", 34, "ms", 0, 60000),
]
# Sensible start values for a fresh tune: (name, value). Pins 255 = not connected, so the gearbox stays off.
GB_DEFAULTS = [("gbEnabled", 0), ("gbDownToNeutral", 0), ("gbCutUp", 1), ("gbCutDown", 0), ("gbCutIgn", 1),
               ("gbCutFuel", 0), ("gbFeedback", 0), ("gbPaddleLow", 1), ("gbNeutralLow", 1),
               ("gbMaxGear", 5), ("gbPinUp", 255), ("gbPinDown", 255), ("gbPinNeutral", 255),
               ("gbCutMinRpm", 3000), ("gbMaxCutMs", 150), ("gbDownMaxRpm", 0), ("gbUpTravel", 800),
               ("gbDownTravel", 800), ("gbPosTol", 50), ("gbDebounce", 20), ("gbCutLead", 15),
               ("gbMoveMs", 80), ("gbHoldMs", 30), ("gbReturnMs", 80), ("gbVerifyMs", 0),
               ("gbMinInterval", 250), ("gbMaxShiftMs", 600), ("gbFaultLock", 1000)]


def gb_constants():
    """The [Constants] lines of page 17."""
    L = ["page = %d" % GB_PAGE_NUMBER, ";Paddle shift gearbox settings (fork addition). Layout = struct gearboxSettingsPage_t, see gearboxSettings.h."]
    for name, byte, bit, t0, t1 in GB_BITS:
        L.append('      %-14s = bits,   U08,  %3d, [%d:%d], "%s", "%s"' % (name, byte, bit, bit, t0, t1))
    for name, typ, byte, unit, lo, hi in GB_NUMS:
        L.append('      %-14s = scalar, %s,  %3d, "%s", 1.0, 0.0, %d, %d, 0' % (name, typ, byte, unit, lo, hi))
    return L


def gb_dialogs():
    """Dialogs of the gearbox page."""
    L = []
    a = L.append
    a('    dialog = gbSwitches, "Switches", yAxis')
    a('        field = "Gearbox controller",            gbEnabled')
    a('        field = "Highest gear",                  gbMaxGear,   { gbEnabled }')
    a('        field = "Downshift from 1st to neutral", gbDownToNeutral, { gbEnabled }')
    a('        field = "Wait for position feedback",    gbFeedback,  { gbEnabled }')
    a('    dialog = gbPins, "Inputs", yAxis')
    a('        field = "Pin numbers: 255 = not connected. Pins already used by Speeduino are refused (gearbox then stays off)."')
    a('        field = "Upshift paddle pin",   gbPinUp,      { gbEnabled }')
    a('        field = "Downshift paddle pin", gbPinDown,    { gbEnabled }')
    a('        field = "Neutral switch pin",   gbPinNeutral, { gbEnabled }')
    a('        field = "Paddle switches",      gbPaddleLow,  { gbEnabled }')
    a('        field = "Neutral switch",       gbNeutralLow, { gbEnabled }')
    a('        field = "Paddle debounce",      gbDebounce,   { gbEnabled }')
    a('    dialog = gbCut, "Engine cut", yAxis')
    a('        field = "Cut on upshift",       gbCutUp,      { gbEnabled }')
    a('        field = "Cut on downshift",     gbCutDown,    { gbEnabled }')
    a('        field = "Cut ignition",         gbCutIgn,     { gbEnabled }')
    a('        field = "Cut fuel",             gbCutFuel,    { gbEnabled }')
    a('        field = "No cut below",         gbCutMinRpm,  { gbEnabled }')
    a('        field = "Maximum cut time",     gbMaxCutMs,   { gbEnabled }')
    a('        field = "Refuse downshift above (0 = no check)", gbDownMaxRpm, { gbEnabled }')
    a('    dialog = gbTiming, "Shift timing", yAxis')
    a('        field = "Cut starts before move", gbCutLead,    { gbEnabled }')
    a('        field = "Move time",              gbMoveMs,     { gbEnabled }')
    a('        field = "Hold time",              gbHoldMs,     { gbEnabled }')
    a('        field = "Return time",            gbReturnMs,   { gbEnabled }')
    a('        field = "Gear check time (0 = off)", gbVerifyMs, { gbEnabled }')
    a('        field = "Minimum time between shifts", gbMinInterval, { gbEnabled }')
    a('        field = "Whole shift watchdog",   gbMaxShiftMs, { gbEnabled }')
    a('        field = "Pause after a fault",    gbFaultLock,  { gbEnabled }')
    a('    dialog = gbActuator, "Actuator", yAxis')
    a('        field = "Travel in permille of full stroke (1000 = full). No servo driver is built in yet."')
    a('        field = "Upshift travel",    gbUpTravel,   { gbEnabled }')
    a('        field = "Downshift travel",  gbDownTravel, { gbEnabled }')
    a('        field = "Position tolerance", gbPosTol,    { gbEnabled && gbFeedback }')
    a('    dialog = gbController, "Gearbox controller (paddle shift)", yAxis')
    a('        field = "Disabled by default. Needs the paddle pins; settings are applied when no shift is running."')
    a('        panel = gbSwitches')
    a('        panel = gbPins')
    a('        panel = gbCut')
    a('        panel = gbTiming')
    a('        panel = gbActuator')
    return L


def gb_menu():
    return ['        subMenu = gbController,      "Gearbox controller"']


def gb_extensions():
    return ["        defaultValue = %s, %d" % (n, v) for n, v in GB_DEFAULTS]


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
    L.extend(gb_menu())
    return L


def block(name, lines):
    return ["; >>> CAN TABLE %s (generated by tools/gen_can_table_ini.py, do not edit by hand) <<<" % name] + lines + \
           ["; >>> END CAN TABLE %s <<<" % name]


def strip_blocks(text):
    return re.sub(r"; >>> CAN TABLE (\w+) .*?; >>> END CAN TABLE \1 <<<\n", "", text, flags=re.S)


def fix_page_lists(text):
    """nPages and the per-page lists get entries for page 16 and 17. The first 15 entries (upstream pages)
    are kept, the rest is rewritten, so running the script again gives the same result."""
    def set_tail(key, tail):
        # tail None = repeat the first item of the list (all pages use the same command)
        def repl(m):
            items = [i.strip() for i in m.group(2).split(",")]
            if len(items) < PAGE_NUMBER - 1:
                sys.exit("list %s has fewer than 15 entries" % key)
            base = items[:PAGE_NUMBER - 1]
            extra = tail if tail is not None else [base[0]] * (N_PAGES - len(base))
            return m.group(1) + ", ".join(base + extra)
        return re.subn(r"^(\s*%s\s*=\s*)(.*)$" % key, repl, text, flags=re.M)

    text = re.sub(r"nPages\s*=\s*\d+", "nPages              = %d" % N_PAGES, text, count=1)
    for key, tail in (("pageSize", [str(PAGE_SIZE), str(GB_PAGE_SIZE)]),
                      ("pageIdentifier", [r'"\$tsCanId\x%02X"' % PAGE_NUMBER, r'"\$tsCanId\x%02X"' % GB_PAGE_NUMBER]),
                      ("pageReadCommand", None), ("pageValueWrite", None), ("pageChunkWrite", None),
                      ("crc32CheckCommand", None), ("burnCommand", None)):
        text, n = set_tail(key, tail)
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
    text = text.replace(marker, "\n".join(block("PAGE", constants())) + "\n" + "\n".join(block("GBPAGE", gb_constants())) + "\n" + marker, 1)

    # dialogs: directly after the [UserDefined] line
    text = text.replace("\n[UserDefined]\n", "\n[UserDefined]\n" + "\n".join(block("DIALOGS", dialogs() + gb_dialogs())) + "\n", 1)

    # menu: directly after the CAN broadcast entry
    anchor = '        subMenu = CanBcast,             "CAN Broadcasting menu", { CANisAvailable }\n'
    if anchor not in text:
        sys.exit("cannot find the CAN menu entry")
    text = text.replace(anchor, anchor + "\n".join(block("MENU", menu())) + "\n", 1)

    # default values: directly after the [ConstantsExtensions] line
    if "\n[ConstantsExtensions]\n" not in text:
        sys.exit("cannot find [ConstantsExtensions]")
    text = text.replace("\n[ConstantsExtensions]\n", "\n[ConstantsExtensions]\n" + "\n".join(block("DEFAULTS", gb_extensions())) + "\n", 1)

    INI.write_text(text)
    print("written", INI)


if __name__ == "__main__":
    main()
