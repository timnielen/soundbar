#!/usr/bin/env python3
"""
pc_control.py - tiny terminal for pc_control.c

    pip install pyserial
    python pc_control.py COM5

The STM32 understands two commands (all numbers hex, 8-bit address):
    w 58 24 00 80 00 00     write bytes from register 0x24 on
    r 58 24 4               read 4 bytes from register 0x24
Everything you type is sent to it, except these local commands:

    save NAME [N]     store the last command (or the last N commands)
                      under NAME; typing NAME later sends them again
    macros            list saved commands
    forget NAME       delete one
    convert b0101     binary -> hex      (prints 0x5)
    convert 0x5       hex -> binary      (prints b0101)
    quit              exit (Ctrl+C works too)

Saved commands are kept in pc_macros.json next to this script, so they
survive a restart. Text after # is a comment, so datasheet scripts can
be pasted as they are.
"""
import json
import sys
import threading
import time
from pathlib import Path

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
MACRO_FILE = Path(__file__).with_name("pc_macros.json")
LOCAL = {"save", "macros", "forget", "convert", "quit"}


# must match MX_USART1_UART_Init(): 115200 baud, 8 data bits, 2 stop bits
s = serial.Serial(PORT, 115200, stopbits=serial.STOPBITS_TWO, timeout=0.1)

history = []                       # every line sent to the board, in order


# ---------------------------------------------------------------- serial

def reader():
    """Background thread: print whatever the STM32 sends."""
    while True:
        data = s.read(256)
        if data:
            sys.stdout.write(data.decode(errors="replace"))
            sys.stdout.flush()


def send(line):
    s.write((line + "\n").encode())
    history.append(line)
    time.sleep(0.02)               # let the reply arrive before the next line


# ---------------------------------------------------------------- macros

def load_macros():
    try:
        return json.loads(MACRO_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def store_macros():
    MACRO_FILE.write_text(json.dumps(macros, indent=2))


def cmd_save(args):
    if not args:
        print("usage: save NAME [N]")
        return
    name = args[0].strip('"\'')
    count = int(args[1]) if len(args) > 1 else 1
    if name in LOCAL:
        print(f"'{name}' is a local command, pick another name")
        return
    if count < 1 or count > len(history):
        print(f"only {len(history)} command(s) sent so far")
        return
    macros[name] = history[-count:]
    store_macros()
    print(f"saved '{name}':")
    for line in macros[name]:
        print("   ", line)


def cmd_macros(_args):
    if not macros:
        print("no saved commands")
    for name, lines in macros.items():
        print(f"{name}: " + " | ".join(lines))


def cmd_forget(args):
    name = args[0].strip('"\'') if args else ""
    if macros.pop(name, None) is None:
        print(f"no command '{name}'")
    else:
        store_macros()
        print(f"forgot '{name}'")


# ---------------------------------------------------------------- convert

def cmd_convert(args):
    if not args:
        print("usage: convert b0101  |  convert 0x5")
        return
    text = args[0].lower().replace("_", "")
    try:
        if text.startswith("0b") or text.startswith("b"):
            value = int(text.lstrip("0").lstrip("b") or "0", 2)
            print(f"0x{value:X}   ({value})")
        elif text.startswith("0x"):
            value = int(text, 16)
            bits = max(4, -(-value.bit_length() // 4) * 4)   # whole nibbles
            print(f"b{value:0{bits}b}   ({value})")
        else:
            print("start with b (binary) or 0x (hex)")
    except ValueError:
        print("not a valid number")


# ---------------------------------------------------------------- main

macros = load_macros()
threading.Thread(target=reader, daemon=True).start()
handlers = {"save": cmd_save, "macros": cmd_macros,
            "forget": cmd_forget, "convert": cmd_convert}

try:
    while True:
        line = input().split("#")[0].strip()
        if not line:
            continue
        word, *args = line.split()
        if word == "quit":
            break
        if word in handlers:
            handlers[word](args)
        elif word in macros:
            for cmd in macros[word]:
                send(cmd)
        else:
            send(line)
except (KeyboardInterrupt, EOFError):
    pass
