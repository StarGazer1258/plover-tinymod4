''' Python conversion of Charley Shattuck's
    TinyMod4 interpreter. This is written with
    the purpose of using a Raspberry Pi Zero (W)
    as a standalone controller for the TinyMod 4.

    As per the original code, this code is in the
    public domain. '''

from plover import log
from plover.machine.base import ThreadedStenotypeBase
from plover.machine.keymap import Keymap
from plover.engine import StenoEngine

from threading import Condition, Event, Thread

import struct
import time
import smbus
import RPi.GPIO as io
io.setmode(io.BCM)

''' This first section is a translation of Charley's
    Forth interprter from C to Python.

    The data stack is for parameter passing.
    This "stack" is circular, like a Green
    Arrays F18A data stack, so overflow
    and underflow are not possible.
    Number of items must be a power of 2

    self._stack[self._p] is Top Of Stack (TOS)
    
    Further reading on th Forth programming language
    can be found at http://forth.org/tutorials.html 
    My personal favorites are http://astro.pas.rochester.edu/Forth/forth.html
    and http://www.exemark.com/FORTH/StartingFORTHfromForthWebsitev9_2013_12_24.pdf '''
class ForthStack():

  def __init__(self, stackSize, stackMask):
    self._stackSize = stackSize
    self._stackMask = stackMask
    self._stack = [0] * self._stackSize
    self._p = 0

  # Push n to top of data stack
  def push(self, n):
    self._p = (self._p + 1) & self._stackMask
    self._stack[self._p] = n

  # Return top of stack
  def pop(self):
    n = self._stack[self._p]
    self._p = (self._p - 1) & self._stackMask
    return n

  # discard top stack item
  def drop(self):
    self.pop()

  # recover dropped stack item
  def back(self):
    for _ in range(1, self._stackSize):
      self.drop()

  # copy top of stack
  def dup(self):
    self.push(self._stack[self._p])

  # exchange two stack items
  def swap(self):
    a = self.pop()
    b = self.pop()
    self.push(a)
    self.push(b)

  # copy second item of stack on top
  def over(self):
    a = self.pop()
    b = self.pop()
    self.push(b)
    self.push(a)
    self.push(b)

  # add top two items
  def add(self):
    a = self.pop()
    self._stack[self._p] = a + self._stack[self._p]

  # bitwise and top two items
  def and_(self):
    a = self.pop()
    self._stack[self._p] = a & self._stack[self._p]

  # inclusive or top two items
  def or_(self):
    a = self.pop()
    self._stack[self._p] = a | self._stack[self._p]

  # exclusive or top two items
  def xor(self):
    a = self.pop()
    self._stack[self._p] = a ^ self._stack[self._p]

  # invert all bits in top of stack
  def invert(self):
    self._stack[self._p] = ~self._stack[self._p]

  # negate top of stack
  def negate(self):
    self._stack[self._p] = -self._stack[self._p]

# Plover plugin that maps input from the TinyMod4 board to Plover Strokes
class TinyMod4Machine(ThreadedStenotypeBase):

  # Layout of keys on board; used by plover in making of self.keymap
  KEYS_LAYOUT = '''
    S1- T- P- H- *1 -F -P -L -T -D
    S2- K- W- R- *2 -R -B -G -S -Z
           A- O- #1 -E -U
  '''

  def __init__(self, _):
    super(TinyMod4Machine, self).__init__()
    keys = self.get_keys()
    self.keymap = Keymap(keys, keys)
    self.keymap.set_mappings(zip(keys, keys))
    self._pressed = False
    self._stack = ForthStack(8, 7)
    self._modeSelectPin = 12

  # Set up and connect to harware
  def _connect(self):
    connected = False

    # Set plover mode to initializing
    self._initializing()

    try:
      # Init the raw pins
      io.setup(self._modeSelectPin, io.IN, pull_up_down = io.PUD_UP)
      io.setup(16, io.IN, pull_up_down = io.PUD_UP)
      io.setup(20, io.IN, pull_up_down = io.PUD_UP)
      io.setup(21, io.IN, pull_up_down = io.PUD_UP)
      io.setup(1, io.IN, pull_up_down = io.PUD_UP)
      io.setup(26, io.IN, pull_up_down = io.PUD_UP)
      io.setup(19, io.IN, pull_up_down = io.PUD_UP)
      io.setup(13, io.IN, pull_up_down = io.PUD_UP)
      io.setup(6, io.IN, pull_up_down = io.PUD_UP)
      io.setup(0, io.IN, pull_up_down = io.PUD_UP)

      # Init the port expander pins
      self._bus = smbus.SMBus(1)
      self._bus.write_byte_data(0x20, 0x0c, 0xff)
      self._bus.write_byte_data(0x20, 0x0d, 0xff)

      # Check if switch on board is set to NKRO
      if io.input(self._modeSelectPin):
        log.warning("NKRO not selected on board. TinyMod4 plugin Disabled.")
        self._error()
        return connected

      self._ready()
      connected = True
      return connected
    except:
      log.warning("Error setting up TinyMod4")
      self._error()

    return connected

  # Reconnect if disconnected
  def _reconnect(self):
    connected = self._connect()

    while not self.finished.isSet() and not connected:
      time.sleep(0.5)
      connected = self._connect()
      return connected


  # Stop listening for strokes
  def stop_capture(self):
    super(TinyMod4Machine, self).stop_capture()
    self._stopped()
  
  # Covert keys to stroke and send to plover for translation
  def _on_stroke(self, keys):
    steno_keys = self.keymap.keys_to_actions(keys)
    if steno_keys:
      self._notify(steno_keys)

  # Read keys from raw pins
  def _read_raw_keys(self):
    a = 0
    a |=  io.input(16)        # 1
    a |= (io.input(20) << 1)  # 2
    a |= (io.input(21) << 2)  # 4
    a |= (io.input(1) << 3)   # 8
    a |= (io.input(26) << 4)  # 10
    a |= (io.input(19) << 5)  # 20
    a |= (io.input(13) << 6)  # 40
    a |= (io.input(6) << 7)   # 80
    a |= (io.input(0) << 8)   # 100
    a ^= 0x01ff
    self._stack.push(a)

  # Read keys from port expander pins
  def _read_ab(self):
    a = self._bus.read_byte_data(0x20, 0x12)
    b = self._bus.read_byte_data(0x20, 0x13)
    a |= b << 8
    a ^= 0xffff
    self._stack.push(a)

  # Read all keys combined
  def _read_all(self):
    self._pressed = True
    self._read_raw_keys()
    self._read_ab()
    self._stack.over()
    self._stack.over()
    self._stack.or_()
    a = self._stack.pop()
    if a == 0:
      self._pressed = False

  # Scan for keys
  def _scan(self):
    while not self._pressed:
      while not self._pressed:
        self._read_all()
      time.sleep(0.03)
      self._read_all()
    a = 0
    b = 0
    while self._pressed:
      self._read_all()
      b |= self._stack.pop()
      a |= self._stack.pop()
    self._stack.push(a)
    self._stack.push(b)

  # Convert and send keys
  def _send(self):
    b = self._stack.pop()
    a = self._stack.pop()

    keys = []

    # Steno order: STKPWHRAO*EUFRPBLGTSDZ
    if a & 0x10:   keys.append('S1-')
    if a & 0x08:   keys.append('S2-')
    if a & 0x20:   keys.append('T-')
    if a & 0x04:   keys.append('K-')
    if a & 0x40:   keys.append('P-')
    if a & 0x02:   keys.append('W-')
    if a & 0x80:   keys.append('H-')
    if a & 0x01:   keys.append('R-')
    if b & 0x08:   keys.append('A-')
    if b & 0x10:   keys.append('O-')
    if a & 0x100:  keys.append('*1')
    if b & 0x200:  keys.append('*2')
    if b & 0x20:   keys.append('#1')
    if b & 0x40:   keys.append('-E')
    if b & 0x80:   keys.append('-U')
    if b & 0x8000: keys.append('-F')
    if b & 0x01:   keys.append('-R')
    if b & 0x4000: keys.append('-P')
    if b & 0x02:   keys.append('-B')
    if b & 0x2000: keys.append('-L')
    if b & 0x04:   keys.append('-G')
    if b & 0x1000: keys.append('-T')
    if b & 0x800:  keys.append('-S')
    if b & 0x100:  keys.append('-D')
    if b & 0x400:  keys.append('-Z')

    if keys:
      self._on_stroke(keys)

  # Start the thread
  def run(self):
    self._ready()
    while not self.finished.isSet():
      self._scan()
      self._send()

# Plover extension that hooks send_string, send_backspaces, and send_key_combination to the HID device
class TinyMod4Extension(Thread):
  
  def __init__(self, engine: StenoEngine):
    global NO_MOD, L_CTRL, L_SHIFT, L_ALT, L_META, R_CTRL, R_SHIFT, R_ALT, R_META
    super().__init__()
    self._engine: StenoEngine = engine
    self._lock = Condition()
    self._stop = Event()
    self._buffer = []

    ''' This massive dictionary is a lookup table
        (LUT) for character codes to HID scan
        codes to be sent to the HID output device.

        It isn't completely extensive, but it does
        include all basic characters on a US ANSI keyboard.
        
        The basic layout of an HID report packet is as follows:
        0       8       16      24      32      40      48      56      64
        -----------------------------------------------------------------
        |Mod Key|Reservd| Key 1 | Key 2 | Key 3 | Key 4 | Key 5 | Key 6 |
        -----------------------------------------------------------------

        The two values in the tuples below are mapped to Mod Key and Key 1 respectively.

        Mod Keys:
        00000000   (0): None
        00000001   (1): Left Control
        00000010   (2): Left Shift
        00000100   (4): Left Alt
        00001000   (8): Left Meta
        00010000  (16): Right Control
        00100000  (32): Right Shift
        01000000  (64): Right Alt
        10000000 (128): Right Meta

        Any combination of these modifiers can be used by bitwise AND-ing their values. (e.g. L_CTRL & R_SHIFT = 00100001 (65))

        More information on this format can be found at: https://wiki.osdev.org/USB_Human_Interface_Devices '''

    # Mod Keys
    NO_MOD  = 0x00
    L_CTRL  = 0x01
    L_SHIFT = 0x02
    L_ALT   = 0x04
    L_META  = 0x08
    R_CTRL  = 0x10
    R_SHIFT = 0x20
    R_ALT   = 0x40
    R_META  = 0x80

    self._hid_lut = {
      "a": (NO_MOD,   4), "b": (NO_MOD,   5), "c": (NO_MOD,   6),
      "d": (NO_MOD,   7), "e": (NO_MOD,   8), "f": (NO_MOD,   9),
      "g": (NO_MOD,  10), "h": (NO_MOD,  11), "i": (NO_MOD,  12),
      "j": (NO_MOD,  13), "k": (NO_MOD,  14), "l": (NO_MOD,  15),
      "m": (NO_MOD,  16), "n": (NO_MOD,  17), "o": (NO_MOD,  18),
      "p": (NO_MOD,  19), "q": (NO_MOD,  20), "r": (NO_MOD,  21),
      "s": (NO_MOD,  22), "t": (NO_MOD,  23), "u": (NO_MOD,  24),
      "v": (NO_MOD,  25), "w": (NO_MOD,  26), "x": (NO_MOD,  27),
      "y": (NO_MOD,  28), "z": (NO_MOD,  29), "A": (L_SHIFT,  4),
      "B": (L_SHIFT,  5), "C": (L_SHIFT,  6), "D": (L_SHIFT,  7),
      "E": (L_SHIFT,  8), "F": (L_SHIFT,  9), "G": (L_SHIFT, 10),
      "H": (L_SHIFT, 11), "I": (L_SHIFT, 12), "J": (L_SHIFT, 13),
      "K": (L_SHIFT, 14), "L": (L_SHIFT, 15), "M": (L_SHIFT, 16),
      "N": (L_SHIFT, 17), "O": (L_SHIFT, 18), "P": (L_SHIFT, 19), 
      "Q": (L_SHIFT, 20), "R": (L_SHIFT, 21), "S": (L_SHIFT, 22),
      "T": (L_SHIFT, 23), "U": (L_SHIFT, 24), "V": (L_SHIFT, 25),
      "W": (L_SHIFT, 26), "X": (L_SHIFT, 27), "Y": (L_SHIFT, 28),
      "Z": (L_SHIFT, 29), "1": (NO_MOD,  30), "2": (NO_MOD,  31),
      "3": (NO_MOD,  32), "4": (NO_MOD,  33), "5": (NO_MOD,  34),
      "6": (NO_MOD,  35), "7": (NO_MOD,  36), "8": (NO_MOD,  37),
      "9": (NO_MOD,  38), "0": (NO_MOD,  39), "!": (NO_MOD,  30),
      "@": (L_SHIFT, 31), "#": (L_SHIFT, 32), "$": (L_SHIFT, 33),
      "%": (L_SHIFT, 34), "^": (L_SHIFT, 35), "&": (L_SHIFT, 36),
      "*": (L_SHIFT, 37), "(": (L_SHIFT, 38), ")": (L_SHIFT, 39),
      "\n":(NO_MOD,  40), "\b":(NO_MOD,  42), "\t":(NO_MOD,  43),
      " ": (NO_MOD,  44), "-": (NO_MOD,  45), "_": (L_SHIFT, 45),
      "=": (NO_MOD,  46), "+": (L_SHIFT, 46), "[": (NO_MOD,  47),
      "{": (L_SHIFT, 47), "]": (NO_MOD,  48), "}": (L_SHIFT, 48),
      "\\":(NO_MOD,  49), "|": (L_SHIFT, 49), ";": (NO_MOD,  51),
      ":": (L_SHIFT, 51), "'": (NO_MOD,  52), "\"":(L_SHIFT, 52),
      "`": (NO_MOD,  53), "~": (L_SHIFT, 53), ",": (NO_MOD,  54),
      "<": (L_SHIFT, 54), ".": (NO_MOD,  55), ">": (L_SHIFT, 55),
      "/": (NO_MOD,  56), "?": (L_SHIFT, 56)
    }
  
  # Called by plover to start the machine
  def start(self):
    log.info("TinyMod4Extension starting...")

    # Hooks that are called by plover when the events occurr
    self._engine.hook_connect("send_string", self._send_string) # Called when a string is to be outputted
    self._engine.hook_connect("send_backspaces", self._send_backspaces) # Called when backspaces are to be outputted
    self._engine.hook_connect("send_key_combination", self._send_key_combination) # Called when a key combination is to be outputted

    log.info("TinyMod4Extension started.")

    super().start()

  # Called by plover to stop the extension
  def stop(self):
    log.info("TinyMod4Extension stopping...")

    self._stop.set()
    self._notify()

    self._engine.hook_disconnect("send_string", self._send_string)
    self._engine.hook_disconnect("send_backspaces", self._send_backspaces)
    self._engine.hook_disconnect("send_key_combination", self._send_key_combination)

    log.info("TinyMod4Extension stopped.")

  # Main loop of the process thread
  def run(self):
    while not self._stop.isSet():
      while len(self._buffer) > 0:
        action = self._buffer.pop()

        if action["type"] == "string":
          # Open the HID device and write the appropriate characters by scan code from the lookup table
          with open("/dev/hidg0", "wb") as f:
            for char in  list(action["string"]):
              try:
                mod, key = self._hid_lut[char]

                # A  struct is used to pack a report for the HID device.
                # The structure iof this report is described above the definiton for self._hid_lut
                f.write(struct.pack("BBBBL", mod, 0x00, key, 0x00, 0x00000000))
                f.write(struct.pack("Q", 0))
              except:
                # If the the character is not found in the lookup table, the plugin instead outputs a ? (scan code 56)
                f.write(struct.pack("BBBBL", NO_MOD, 0x00, 56, 0x00, 0x00000000))
                f.write(struct.pack("Q", 0))
        elif action["type"] == "backspace":
          # Open the HID devices and write the appropriate number of backspace characters (scan code 42)
          with open("/dev/hidg0", "wb") as f:
            for b in range(action["backspace"]):
              f.write(struct.pack("BBBBL", NO_MOD, 0x00, 42, 0x00, 0x00000000))
              f.write(struct.pack("Q", 0))
        elif action["type"] == "keycombo":\
          # Not yet implemented, requires large lookup tables to map the keys
          log.info("send_keycombo: " + str(action["keycombo"]))

      # Block the thread until another event has occured
      self._wait()

  # Make thread wait for next event
  def _wait(self):
    with self._lock:
      self._lock.wait()

  # Notify the thread that an event has occured
  def _notify(self):
    with self._lock:
      self._lock.notify()

  # Add a string object to the output buffer to be processed
  def _send_string(self, s):
    self._buffer.insert(0, {"type": "string", "string": s})
    self._notify()

  # Add a backspace object to the output buffer to be processed
  def _send_backspaces(self, b):
    self._buffer.insert(0, {"type": "backspace", "backspace": b})
    self._notify()

  # Add a key combination object to the output buffer to be processed
  def _send_key_combination(self, c):
    self._buffer.insert(0, {"type": "keycombo", "keycombo": c})
    self._notify()

  # Unused debug function, used to manually send data to the HID device
  def _send_hid_report(self, report):
    with open('/dev/hidg0', 'rb+') as fd:
      fd.write(report.encode())