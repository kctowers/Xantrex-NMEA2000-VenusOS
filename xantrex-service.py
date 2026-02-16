#!/usr/bin/env python3
# Version: 1.0.0.2026.01.25
# Date: 2026-01-25

# Xantrex Freedom Pro NMEA 2000 D-Bus Driver
#

# This script reads raw NMEA-2000 (CAN) data from a Xantrex Freedom Pro Marine inverter/charger
# and publishes meaningful decoded values to the Venus OS D-Bus using com.victronenergy.inverter.can_xantrex.
# 
# Author: Kevin Towers
#
# This was based on the RV-C driver created by Scott Sheen
# It can be found at https://github.com/ScottS45/Xantrex-Rvc-VenusOS
#
# Compatible with: Venus OS (standard installations with dbus, GLib, and python3)
# Requirements: Connected CAN interface (default: vecan0) and appropriate udev/socket permissions
#
# Features:
# - Decodes known Xantrex NMEA-2000 PGNs and maps them to Victron D-Bus paths
# - Auto-registers standard D-Bus monitoring paths for inverter/chargers under /Mgmt/xantrex-nmea
# - Supports logging flags (--log,  --verbose)
# - Includes /Status reporting, derived power calculations, and GLib heartbeat timers
#
#
# Packet decoding information came from the following web pages:
# https://xantrex.com/wp-content/uploads/2021/12/976-0422-01-01_Rev-ANMEA2000-PGN-List-for-FXCC_ENG.pdf
# https://canboat.github.io/canboat/canboat.html
# Also from lot of trial and error testing and reverse engineering CAN packets.
#
# Based on Venus OS community conventions (e.g., GuiMods, dbus-serialbattery)
#

# --- Venus OS Compatibility Notes ---
# This service was developed on Venus OS (Large) V3.67
#
# This code supports a single instance of a Xantrrex inverter on the CAN bus.
# Because of this, we don't care what node ID any of the data packets come
# from.  If you want to support multiple instances, future upgrades my include
# commandline options to associate CAN Id with instance numbers.

# --- Import standard and system libraries ---
import sys
import socket
import struct
import os
import errno
import logging
import signal
import time
import re
import enum
import dbus
import dbus.mainloop.glib

from functools     import partial
from collections   import defaultdict
from typing        import Any, Optional, Set, Dict, List
from gi.repository import GLib
import argparse


# ─── Load our locally vendored Velib Python library ───

# We have manually placed the velib_python package under /data/xantrex-monitor/velib_python so that:
#  • We control the exact Vedbus implementation (tested for no root-“/” binding)
#  • It lives in /data and survives both reboots and firmware updates
#  • We can safely instantiate two VeDbusServiceWithMeta services on one bus
#
# This can be found at https://github.com/victronenergy/velib_python
#
# Prepending this path ensures all “import vedbus” calls use our vendored copy first.
sys.path.insert(0, '/data/xantrex-monitor/velib_python')
from vedbus import VeDbusItemImport
import vedbus

# --- Constants defining service identity and CAN parameters ---
DEVICE_INSTANCE        = 0                      # This driver only supports a single instance, so hardcoded here
PRODUCT_ID             = 1234
FIRMWARE_VERSION       = '2.14'    # hard coded, matches mine.
XANTREX_INVERTER_MODEL = 2000                   # 2000 or 3000 Watt inverter
PRODUCT_NAME           = 'Freedom XC Pro ' + str(XANTREX_INVERTER_MODEL) + ' Marine'
SCRIPT_VERSION         = '1.0.0.2026.01.25'
MAX_UNMAPPED_PGNS      = 100

# ManufacturerCode = 119

INV_CHG_STATE = {
    0: 0,   #  NA/init       → Off
    1: 0,   #  Not charging  Maybe this needs to be 1: 1
    2: 3,   #  Bulk
    3: 4,   #  Absorption
    4: 4,   #  Absorption
    5: 7,   #  Equalize
    6: 5,   #  Float
    7: 7,   #  Equalize
    8: 8,   #   — passthru     → Passthru
    9: 9,   #   — inverting    → Inverting
    10: 10, #   — assisting    → Assisting
    11: 11, #   — psu          → Power supply
}
      
# These are values that the GUI will send down to change the operating mode of
# Inverter/Charger
MODE_CHARGER = 1
MODE_INVERTER = 2
MODE_ON = 3                                     # operates in both inverter and charger modes
MODE_OFF = 4

# === CLI Argument Parsing ===
parser = argparse.ArgumentParser(allow_abbrev = True)
parser.add_argument('--log', default='WARNING', 
    help='Set the logging level (Default: WARNING)')
parser.add_argument('--verbose', action='store_true', help='Print all logging to terminal')
parser.add_argument('--can',     default='vecan0',     metavar='IFACE',  help='SocketCAN interface to listen on (default: vecan0)')
args = parser.parse_args()

CAN_INTERFACE = args.can
RCVBUF_BYTES  = 1 * 1024 * 1024          # we want 1 MiB payload room


# === Configure Logging ===
# make the dir if it is not there
# remove the existing log to start fresh.  
LOG_DIR = '/data/xantrex-monitor/logs'
os.makedirs(LOG_DIR, exist_ok=True)

logger = logging.getLogger("xantrex")
logger.handlers.clear()                         # remove any inherited handlers
logger.propagate = False                        # don’t duplicate to root logger 
# choose level
logger.setLevel (args.log.upper())              # level comes from command line

fh = logging.FileHandler(f'{LOG_DIR}/xantrex.log', mode='w') # 'w' forces a overwrite if file already exists
fh.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

logger.addHandler(fh)                           # always log to a file
# and if requested, output everything to the terminal as well
if args.verbose:
    fh = logging.StreamHandler(sys.stdout)
    logger.addHandler(fh)

    
# ─── SET UP D-BUS TO USE THE GLIB MAIN LOOP ───
# We need D-Bus events (method calls, signals, introspection requests)
# to be dispatched via GLib so they integrate with our CAN I/O loop.
# Calling DBusGMainLoop(set_as_default=True) here ensures that *any*
# subsequent BusConnection (including private ones) will use GLib.
# This must appear *before* creating any BusConnection, hence its
# placement at the top of the module.
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    
# Create a shared GLib-backed loop instance for all private connections
glib_loop = dbus.mainloop.glib.DBusGMainLoop()
   
def new_system_bus():
    # Return a private SystemBus connection that dispatches on our shared GLib loop, so multiple VeDbusService instances don’t collide on '/'.
    return dbus.SystemBus( mainloop = glib_loop, private = True )
    


    
# === Supported PGNs used by Venus OS ===
# ------------------------------------------------------------------------------
# Decoder functions for the PGN_MAPs that safely extract numeric values from CAN data.
# These functions automatically return None if the raw value is masked or invalid
# according to RV-C and J1939 standards:
#   - 0xFF       for 8-bit fields
#   - 0x7F       for signed 8-bit fields
#   - 0xFFFF     for 16-bit fields
#   - 0x7FFF/-32768 for signed 16-bit fields
#   - 0xFFFFFFFF for 32-bit fields
# This ensures masked data is skipped during D-Bus updates instead of being logged
# as large incorrect values (e.g., 6553.5 A or 4294967295 W).
# ------------------------------------------------------------------------------


# ── Pre-compiled struct formats ────────────────────────────────────────────
# A Struct caches parsing metadata in C, so calling the bound .unpack_from()
# avoids reparsing the format string on *every* frame.
# Each returns a tuple ⇒ we unpack with the trailing “,”.
_UNPACK_U8  = struct.Struct('<B').unpack_from
_UNPACK_S8  = struct.Struct('<b').unpack_from
_UNPACK_U16 = struct.Struct('<H').unpack_from
_UNPACK_S16 = struct.Struct('<h').unpack_from
_UNPACK_U32 = struct.Struct('<I').unpack_from
_UNPACK_S32 = struct.Struct('<i').unpack_from

# Big-endian versions
_UNPACK_U16_BE = struct.Struct('>H').unpack_from
_UNPACK_S16_BE = struct.Struct('>h').unpack_from

# UNSIGNED DECODERS
#   Return None when the spec defines the value as "not available"

def safe_u8(data: bytes | memoryview, offset: int, scale: float = 1.0) -> Optional[float]:
    # Unsigned 8-bit. 0xFF ⇒ NA per RV-C.
    if len(data) <= offset: return None
        
    value = data[offset]
    if value == 0xFF: return None
    result = value * scale
    return round(result, 3) if scale != 1.0 else result

def safe_u16(data: bytes | memoryview, offset: int, scale: float = 1.0, byteorder: str = "little") -> Optional[float]:
    # Unsigned 16-bit LE and BE. 0xFFFF ⇒ NA per RV-C.
    if len(data) < offset + 2:
        return None
        
    if byteorder == "little":
        raw, = _UNPACK_U16(data, offset)
    else:
        raw, = _UNPACK_U16_BE(data, offset)
    
    if raw == 0xFFFF:  # NA sentinel for u16
        return None
    if scale == 1.0:
        return raw  # Return integer directly  
        

    return round(raw * scale, 3)  # Return scaled and rounded float    
    

def safe_u32(data: bytes | memoryview, offset: int, scale: float = 1.0) -> Optional[float]:
    # Unsigned 32-bit LE. 0xFFFFFFFF ⇒ NA per RV-C.
    if len(data) < offset + 4: return None
        
    raw, = _UNPACK_U32(data, offset)
    
    if raw == 0xFFFFFFFF:  # NA sentinel for u32
        return None
    if scale == 1.0:
        return raw  # Return integer directly

    return round(raw * scale, 3)  # Return scaled and rounded float
        


# SIGNED DECODERS
#   Return None when the spec defines the value as "not available"
def safe_s8(data: bytes | memoryview, offset: int, scale: float = 1.0) -> Optional[float]:
    # Signed 8-bit. 0x7F ⇒ NA per RV-C.
    if len(data) <= offset: return None
        
    value = data[offset]
    if value == 0x7F: return None
        
    signed = value - 256 if (value & 0x80) else value
    result = signed * scale
    
    return round(result, 3) if scale != 1.0 else result

def safe_s16(data: bytes | memoryview, offset: int, scale: float = 1.0, byteorder: str = "little") -> Optional[float]:
    # Signed 16-bit LE and BE. 0x7FFF ⇒ NA per RV-C.
    if len(data) < offset + 2: return None
        
    if byteorder == "little":
        raw, = _UNPACK_S16(data, offset)
    else:
        raw, = _UNPACK_S16_BE(data, offset)
        
    if raw == 0x7FFF or raw == -1:   # -1 == 0xFFFF
        return None
        
    return raw if scale == 1.0 else round(raw * scale, 3)
    

# OTHER DECODERS
    
def safe_s24(data: bytes | memoryview, offset: int, scale: float = 1.0) -> Optional[float]:
    # Signed 24-bit LE. 
    if len(data) < offset + 3: return None  
    
    value = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16)

    # Sign-extend if the 24th bit is set
    if value & 0x800000:
        value -= 1 << 24

    return round(value * scale, 3)

def safe_bit(byte_val: int, mask: int) -> Optional[bool]:
    # Return True if the bitmask is set, False if clear, or None if the whole byte is marked Not-Available. 
    
    if byte_val == 0xFF:            # RV-C “not available”
        return None
        
    return bool(byte_val & mask)
    
    
def safe_ascii(data_slice):
    if all(b == 0xFF for b in data_slice):
        return None
    try:
        cleaned = bytes(data_slice).rstrip(b'\xFF')  # <-- convert then rstrip
        return cleaned.decode('ascii').strip()
    except UnicodeDecodeError:
        return None       
        
def fahrenheit_to_c(val):
    return None if val is None else round((val - 32) * 5/9, 1)


class NMEA2000:
    # not sure what module this was to come from, but we need it to send a 29-bit CAN frame
    CAN_EFF_FLAG        = 0x80000000

    # Define our NMEA transmit packet here 
    # Fields without a 'value' key will pull from the function arguments
    # Data types we support are: uint8, int8, uint16, int16, uint32, int32,
    # string, and bytes
    # ******************************************************************
    # Note these inverter/charger control packets don't work yet.
    # Still hoping for help from Xantrex Tech support.  These are based on
    # info from the canboat reverse engineered NMEA data project
    # ******************************************************************
    TRANSMIT_PGN_MAP = {
        'CONTROL_CHARGER': {                        # enable/disable charger packet
            'PRI': 6,                               # priority 
            'PGN': 0x1F216,                         # PGN
            'FP': True,                             # is this a fast packet?
            'DATA': {
                'instance':                 {'type': 'uint8', 'value': 1},
                'DC_instance':              {'type': 'uint8', 'value': 1},
                'enable':                   {'type': 'uint8'},         # Variable (Param 1)
                'charge_current_limit':     {'type': 'uint8', 'value': 100},  # percent 
                'algorithm_mode':           {'type': 'uint8', 'value': 0}, 
                'flags':                    {'type': 'uint8', 'value': 0}, 
                'equalize_timet':           {'type': 'uint16', 'value': 0}, 
            }                                          
        },
        'CONTROL_INVERTER': {                       # enable/disable inverter packet
            'PRI': 6,                               # priority 
            'PGN': 0x1F217,                         # PGN
            'FP': True,                             # is this a fast packet?
            'DATA': {
                'instance':                 {'type': 'uint8', 'value': 1},
                'AC_instance':              {'type': 'uint8', 'value': 2},
                'DC_instance':              {'type': 'uint8', 'value': 1},
                'enable':                   {'type': 'uint8'},         # Variable (Param 1)
                'load_sense_threshold':     {'type': 'uint16', 'value': 25}, 
                'load_sense_interval':      {'type': 'uint16', 'value': 200}, 
            }                                          
        },     
        # This packet works as it should
        'ACK_ALARM': {                              # send acknowledge packet 
            'PRI': 6,                               # priority
            'PGN': 0x1F008,                         # PGN
            'FP': True,
            'DATA': {
                'response': {'type': 'bytes'},      # This will be a copy of the alarm packet received
                'ack':      {'type': 'uint8'}       # This is the acknowledge type (silence, acknowledge) 
            }
        },
    }    


    def __init__(self, can_address: int) -> None:
        self._rx_fpa = {                        # used to store Fast Packet assembly metadata
            'last_seq': 0,                      # last sequence number seen 
            'bytes_remaining': 0,               # number of bytes remaining to assemble
            'buf': [],                          # accumlator of all data bytes
            'assembling': False,                # flag to indicate whether we're in the middle of things
            }                       
        self._rx_callback = None
        self._our_address = can_address         # save our CAN ID
        self._tx_fpa_sequence = 0               # transmit sequence number for fast packets
        self._can_filters = []                  # list of CAN ID we accept
        
        logger.info(f'Initializing NMEA 2000 Class on address: {can_address}')
          
        # Validate, Open, and bind CAN socket
        # We are using the Linux socketCAN driver here
        # https://www.kernel.org/doc/Documentation/networking/can.txt
        try:
            if not os.path.exists(f'/sys/class/net/{CAN_INTERFACE}'):
                logger.error(f"Interface {CAN_INTERFACE} not found in sysfs")
                sys.exit(1)

            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            
            # --------- enlarge receive queue ------------------------------------
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF_BYTES)
            
            # feedback for the buffer size change
            effective = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            logger.info("CAN receive buffer requested=%d, effective=%d (kernel reports doubled value)", RCVBUF_BYTES, effective)
            
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0)     # off → no echo generated at all

            self.socket.bind((CAN_INTERFACE,))
            logger.info(f"CAN socket bound to {CAN_INTERFACE}")
                
        except OSError as e:
            if e.errno == errno.ENODEV:
                logger.error(f"CAN interface {CAN_INTERFACE} not found (ENODEV)")
            elif e.errno == errno.EPERM:
                logger.error(f"Permission denied: try running as root (EPERM)")
            else:
                logger.error(f"Failed to bind CAN socket on {CAN_INTERFACE}: {e}")
            raise

        # Add socket listener to GLib event loop
        self.watch_id = GLib.io_add_watch(self.socket.fileno(), GLib.IO_IN, self._rx_can_frame)
            
    # This mehtod registers the callback for any CAN frames received.
    def register_rx_callback(self, rx_cb_func) -> None:
        self._rx_callback = rx_cb_func

    # This function adds a filter to the CAN driver.  The PGN passed to this
    # function will be added to a list of PGNs that will be accepted and pass
    # back tot he application.  The filter will accept the PGN from any source
    # address and with any priority.
    def add_can_filter(self, pgn: int) -> None:
        PGN_MASK = 0x3FFFF00                    # Any source and any Priority

        pgn = pgn << 8                          # shift into position above the src address
        # Add the new filter to our local tracking list
        self._can_filters.append((pgn, PGN_MASK))

        # Pack the filters into a binary string
        # Each filter is 2 unsigned 32-bit integers (8 bytes total)
        # 'I' is for 32-bit unsigned int. We use '=' for standard alignment.
        filter_data = b""
        for f_id, f_mask in self._can_filters:
            filter_data += struct.pack("=II", f_id, f_mask)

        # Apply the updated list to the socket
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filter_data)
            logger.info(f"Filter added: ID 0x{pgn:X}. Total: {len(self._can_filters)}")
        except OSError as e:
            logger.error(f"Failed to set filters: {e}")

        
    # send a request to the unit
    def send_can_frame(self, pri: int, pgn: int, data: bytes ) -> bool:
        try:
            sa      = (self._our_address & 0xFF)                # Source Address (SA) 
            can_id  = (pri << 26) | (pgn << 8) | sa             # 29-bit CAN ID: Priority + PGN + source address
            can_id |= self.CAN_EFF_FLAG                         # mark this as a extended frame ID 
            
            # We need to format a C structure for the call to socket.send().  The structure has the format:
            # struct can_frame {
            #     canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
            #     __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
            #     __u8    __pad;   /* padding */
            #     __u8    __res0;  /* reserved / padding */
            #     __u8    __res1;  /* reserved / padding */
            #     __u8    data[8] __attribute__((aligned(8)));
            # };
            frame      = struct.pack("=IB3x", can_id, len(data)) + data    # Full CAN frame
        
            self.socket.send(frame)

            logger.info(f"Sent CAN frame 0x{pgn:05X}")
            return True

        except Exception as e:
            logger.error(f"Failed to send CAN frame 0x{pgn:05X}: {e}")
            return False
                  
    # This method breaks a larger packet into multiple frames
    # using the NMEA "Fast Frame Format".  It directly transmits the frames as
    # they are built.  Note: some NMEA Fast-packet frames payloads will still
    # fit into a single CAN packet, but they need to be sent as FP.  So we
    # assume that the shortest allowable packet is one where there are actually
    # some payload bytes in the second frame (i.e. shortest = 6 bytes)
    def send_fast_packet(self, pri: int, pgn: int, payload: bytes) -> bool:
        if len (payload) > 6:
            idx = 0                             # index into the payload packet
            self._tx_fpa_sequence += 1           # use the next sequence number
            seq = ((self._tx_fpa_sequence) << 4) & 0xff   # sequence number goes into the upper nibble in a local copy
            data = bytearray (8)                # allocate our packet
            
            # send first CAN frame
            data[0] = seq                       # sequence number goes into first byte
            data[1] = len (payload)             # payload length into the second byte
            data[2:8] = payload[:6]             # first 6 bytes
            self.send_can_frame (pri, pgn, data)

            remaining_data = payload[6:]
            # Process the rest in chunks of 7
            # If the last chunk is shorter than 7 (uneven), slicing handles it naturally
            for i in range(0, len(remaining_data), 7):
                seq += 1
                data = bytearray(b'\xff' * 8)   # pad with 0xFF
                data[0] = seq
        
                chunk = remaining_data[i : i + 7]
                data[1 : 1 + len(chunk)] = chunk 
                self.send_can_frame (pri, pgn, data)
                
            return True
        else:
            return False

    # This method sends a NMEA packet defined above.  Optional parameters will
    # need to be passed if the packet format requires them
    def send_nmea_message(self, pkt_key, *args):
        tx_pkt = self.TRANSMIT_PGN_MAP[pkt_key]
        if (tx_pkt):
            packed_data = bytearray()
            arg_list = list(args)
            arg_idx = 0
            # '<' ensures Little-Endian (Standard for most modern hardware)
            ENDIAN = "<"

            pri = tx_pkt.get ("PRI")
            pgn = tx_pkt.get ("PGN")
            fast_packet = tx_pkt.get ("FP")
            
            for key, spec in tx_pkt["DATA"].items():
                data_type = spec.get("type")

                # Determine if we use the static value or a passed parameter
                if "value" in spec:
                    value = spec["value"]
                else:
                    if arg_idx < len(arg_list):
                        value = arg_list[arg_idx]
                        arg_idx += 1
                    else:
                        raise ValueError(f"Missing parameter for variable field: {key}")

                if data_type == "uint8":
                    packed_data.extend(struct.pack(f"{ENDIAN}B", value))
                elif data_type == "int8":
                    packed_data.extend(struct.pack(f"{ENDIAN}b", value))
                elif data_type == "uint16":
                    packed_data.extend(struct.pack(f"{ENDIAN}H", value))
                elif data_type == "int16":
                    packed_data.extend(struct.pack(f"{ENDIAN}h", value))
                elif data_type == "uint32":
                    packed_data.extend(struct.pack(f"{ENDIAN}I", value))
                elif data_type == "int32":
                    packed_data.extend(struct.pack(f"{ENDIAN}i", value))
                elif data_type == "string":
                    encoded = value.encode('utf-8')
                    packed_data.extend(struct.pack(f"{len(encoded)}s", encoded))
                elif data_type == "bytes":
                    packed_data.extend(value)
                else:
                    raise TypeError(f"Unsupported data type: {data_type}")

            # now transmit the frame depending on the frame type
            if fast_packet:
                return self.send_fast_packet (pri, pgn, packed_data)
            else:                   
                return self.send_can_frame (pri, pgn, packed_data)
        else:
            logger.error(f"Unknown transmit packet key {pkt_key}:")
            

    # Only CAN frames that are declared part of a multiple frame message will
    # be passed to this function
    def AssembleFastPacket (self, data: bytes) -> tuple [bool, bytes]:
        # LS nibble of the first byte will always be zero
        if data[0] & 0x0f == 0:
            self._rx_fpa["last_seq"] = data[0]
            self._rx_fpa["bytes_remaining"] = data[1] - 6
            self._rx_fpa["buf"] = bytearray(data[2:8])
            self._rx_fpa["assembling"] = True        
        else:                          
            # are we still assembling and is the sequence number match what we
            # expect
            if self._rx_fpa["assembling"] and (self._rx_fpa["last_seq"] + 1) == data[0]:
                self._rx_fpa["last_seq"] = data[0]
                bytes = self._rx_fpa["bytes_remaining"]
                bytes_to_copy = min(7, bytes) 
                self._rx_fpa["buf"] += data[1:(bytes_to_copy + 1)]      
                self._rx_fpa["bytes_remaining"] -= bytes_to_copy
                if (bytes <= 7):                # we're on the last packet
                    self._rx_fpa["assembling"] = False
                    return True, self._rx_fpa["buf"]
                    
        return False, []            
            
    # Format CAN frame for logging
    def format_can_frame(self, PGN, data):
        hexdata = ' '.join(f'{b:02X}' for b in data)
        return f"PGN=0x{PGN:05X} | DLC={len(data)} | Data={hexdata}"

    # Process a single incoming CAN frame, decode its NMEA data, and update D-Bus paths.
    # Args:
    #    source: File descriptor of the CAN socket.
    #    condition: GLib IO condition (e.g., GLib.IO_IN).
    # Returns:
    #    bool: True to continue processing, False to stop.

    def _rx_can_frame(self, source, condition):
        # === Extract and Decode CAN ID and Data ===
        try:
            # Receive and Parse a J1939/RV-C CAN Frame ===
            # Receive one CAN frame, decode it via RV-C PGN maps, and publish to D-Bus.
            frame = self.socket.recv(16)
            
            # Validate minimum CAN header (8 bytes)
            if len(frame) < 8:
                raise ValueError(f"Received too short CAN frame header: {len(frame)} bytes")
                
            # Extract CAN ID (29-bit extended ID) and DLC (Data Length Code)
            # Format: =IB3x → 4 bytes CAN ID (uint32), 1 byte DLC, skip 3 padding bytes
            can_id, can_dlc = struct.unpack("=IB3x", frame[0:8])
            
            # Use available data, even if less than DLC  
            available_dlc = min(can_dlc, len(frame) - 8)
            if available_dlc == 0:
                logger.warning(f"[NO DATA] Frame=0x{can_id:08X} | PGN=0x{(can_id & 0x1FFFF) >> 8:05X} | DLC={can_dlc} | No data bytes available")
                return True            
                
            # Slice out the actual CAN data payload (up to 8 bytes)
            data = memoryview(frame[8:8 + available_dlc])

            
            # === Decode CAN ID into J1939 / NMEA fields ===
            # According to J1939, a 29-bit CAN ID has:
            #
            #   | Priority (3) | Reserved (1) | Data Page (1) | PDU Format (8) | PDU Specific (8) | Source Address (8) |
            #     <-----------  bits 26–28  -> <-------- bits 24–25, 16–23, 8–15 ----------> <----- bits 0–7 --------->
            #
            # PGN (Parameter Group Number) spans bits 8–25 = 18 bits
            # SRC (SouRCe Address) = bits 0–7
            pgn = (can_id >> 8) & 0x3FFFF   # Extract PGN from bits 8–25 (18 bits)
            src = can_id & 0xFF             # Extract Source Address from bits 0–7

            logger.debug(f"[CAN ID] 0x{can_id:08X} → PGN=0x{pgn:05X} SRC=0x{src:02X} DLC={can_dlc} DATA=[{data.hex(' ').upper()}]")

            # call the registered handler for this frame
            if self._rx_callback:
                self._rx_callback(pgn, src, can_dlc, data)
            return True

        except (OSError, ValueError) as e:
            logger.error(f"[RECV ERROR] Failed to read from CAN socket: {e}")
            return True



# === D-Bus Service Class ===
class XantrexService:
    def __init__(self):
        self.known_alarms = [ 2, 3 ]            # temporary to help flush out unknown alarm codes
        self.unknown_alarm_logged = []          # which alarms have we already captured
        
        # === PGN Map: Decoders from NMEA PGNs to D-Bus paths ===
        # Format: PGN : [(dbus_path, decode_function), ...]
        # Each PGN (Diagnostic Group Number) corresponds to a specific NMEA data packet
        # The lambda decoders extract meaningful values (voltage, current, state, etc.) from the binary payload
        # Units and scaling factors are defined by the NMEA specification or device-specific implementation

        # ─────────────────────────────────────────────────────────────────────────────
        #  INVERTER_PGN_MAP  
        # ─────────────────────────────────────────────────────────────────────────────
        self.INVERTER_RX_PGN_MAP = {
            # PGN 127750 - Converter Status
            0x1F306: {  
                'Fast_Packet': False,
                'SPNs' : [  
                    ('/State',                     lambda d: INV_CHG_STATE.get(int(safe_u8(d, 2) or 0), 0),    '',     'Charger operating state'),
                ]
            },
            # PGN 130900 - Xantrex Proprietary packet
            0x1FF54: {
                'Fast_Packet': True,
                'SPNs': [
                    # AC input values
                    ('/Ac/In/L1/V',                lambda d: safe_s16(d, 5, 0.01)  if d[2] == 1 else None,     'V',     'AC Input Voltage'),
                    ('/Ac/In/L1/I',                lambda d: safe_s16(d, 7, 0.1)   if d[2] == 1 else None,     'A',     'AC Input L1 Current'),
                    ('/Ac/In/L1/F',                lambda d: safe_u16(d, 9, 0.01)  if d[2] == 1 else None,     'Hz',    'AC Input Frequency'),

                    # AC output values
                    ('/Ac/Out/L1/V',               lambda d: safe_s16(d, 5, 0.01)  if d[2] == 2 else None,     'V',     'AC Output Voltage'),
                    ('/Ac/Out/L1/I',               lambda d: safe_s16(d, 7, 0.1)   if d[2] == 2 else None,     'A',     'AC Output L1 Current'),
                    ('/Ac/Out/L1/F',               lambda d: safe_u16(d, 9, 0.01)  if d[2] == 2 else None,     'Hz',    'AC Output Frequency'),
                    ('/Ac/Out/L1/P',               lambda d: safe_u32(d, 11)       if d[2] == 2 else None,     'W',     'AC Output Real Power'),
                ]                                                                                                                         
            },
            # PGN 126983 Alert
            0x1F007: {
                'Fast_Packet': True,
                'SPNs': [  
                ('/Alarms/LowVoltage',             lambda d: self.alarm(d) if d[3] == 2 else None,   '',     'Low Battery Voltage Alarm'),
                ('/Alarms/HighVoltage',            lambda d: self.alarm(d) if d[3] == 3 else None,   '',     'High Battery Voltage Alarm'),

                # These are stubbed out - hope one day to find out what the
                # codes are for them
                ('/Alarms/HighTemperature',        lambda d: self.unknown_alarm(d) if d[3] not in self.known_alarms else None,   '',     'Inverter over temperature'),
                ('/Alarms/Overload',	           lambda d: self.unknown_alarm(d) if d[3] not in self.known_alarms else None,   '',     'Inverter output overloaded'),
                ]
            },
            # PGN 127508 Battery Status
            0x1F214: {
                'Fast_Packet': False,
                'SPNs': [  
                ('/Dc/0/Voltage',                  lambda d: safe_u16(d, 1, 0.01),     'V',     'DC Battery Voltage'),
                ]
            },
            # PGN 127751 - DC Voltage/Current
            0x1F307: {
                'Fast_Packet': False,
                'SPNs': [  
                ('/Dc/0/Current',                  lambda d: safe_s24(d, 4, 0.01),    'A',     'DC Inverter/Charger Current'),
                ]
            }
        }


        # Runtime counters and internal state
        self.frame_count         = 0            # Total CAN frames received
        self.error_count         = 0            # Total decode errors
        self.loop                = None         # Reference to the GLib main loop        
        self.last_heartbeat      = time.time()  # Timestamp of last valid frame received
        self.heartbeat_counter   = 0            
        self.isthereaframe       = 0            
        
        logger.info(f"Initializing Xantrex Service on {CAN_INTERFACE}")

        # This script creates a D-Bus service for the Xantrex Freedom Pro, a single unit with inverter and charger functions:
        self.system_bus = new_system_bus()
        self._InverterService = vedbus.VeDbusService('com.victronenergy.inverter.can_xantrex', bus=self.system_bus, register=True)
        self._InverterService.descriptor = 'INVERTER/CHARGER'
                                    
        # Register all known D-Bus paths defined in the PGN maps (structure only, no data decoding).
        # This ensures all expected paths are visible in D-Bus tools (like dbus-spy) from startup,
        # with correct metadata (unit and description), even before any CAN data is received.
                          
        # ── Register PGN-maps paths ─────────────────────────────────────
        for PGN, PGN_items in self.INVERTER_RX_PGN_MAP.items():
            for item in PGN_items['SPNs']:  # (path, decoder, unit, description)         
                path, decoder, unit, description = item
                # Register the D-Bus path with placeholder value and metadata (if available)
                self._InverterService.add_path(path, None, writeable=False)
            
        # Register /Mgmt/xantrex-can paths on both services                                         
        self._InverterService.add_path('/FirmwareVersion',         FIRMWARE_VERSION,          writeable=False)
        self._InverterService.add_path('/ProductId',               PRODUCT_ID,                writeable=False)
        self._InverterService.add_path('/DeviceInstance',          DEVICE_INSTANCE,           writeable=False)
        self._InverterService.add_path('/Connected',               1,                         writeable=False)
        self._InverterService.add_path('/Status',                  'initializing',            writeable=False)
        self._InverterService.add_path('/Error',                   0,                         writeable=False)
        self._InverterService.add_path('/Mgmt/ProcessAlive',       1,                         writeable=False)
        self._InverterService.add_path('/Mgmt/LastUpdate',         '',                        writeable=False)
        self._InverterService.add_path('/Mgmt/ManufacturerCode',   119,                       writeable=False)
        self._InverterService.add_path('/Mgmt/ProcessName',        'xantrex_nmea',            writeable=False)
        self._InverterService.add_path('/Mgmt/ProcessVersion',     SCRIPT_VERSION,            writeable=False)
        self._InverterService.add_path('/Mgmt/ProcessInstance',    0,                         writeable=False)
        self._InverterService.add_path('/Mgmt/Connection',         f"CAN@{CAN_INTERFACE}",    writeable=False)                               
        self._InverterService.add_path('/Mgmt/Type',               'inverter',                writeable=False)
                                                                                                                 
        self._InverterService.add_path('/CustomName',     PRODUCT_NAME,                       writeable=False)
        # /ProductName is listed as a mandatory path for product services in the D-Bus API (along with /ProductId, /FirmwareVersion, /DeviceInstance, /Connected).
        # This is what shows in the device list.
        self._InverterService.add_path('/ProductName',    PRODUCT_NAME + ' (Inverter)',       writeable=False)
                                                                                                                     
        self._InverterService.add_path('/IsInverterCharger',         1,                       writeable=False)
        self._InverterService.add_path('/Capabilities/HasAcPassthroughSupport', 1,            writeable=False)
        
        # Register additional derived D-Bus paths
        # These are not mapped directly from PGNs but are calculated at runtime
        # includes meta data units and description
        self._InverterService.add_path('/Dc/Instance',              0,                         writeable = False)
        self._InverterService.add_path('/Dc/0/Power',               0,                         writeable = False)

        # /Mode can be changed by the GUI to affect inverter operation 
        self._InverterService.add_path('/ModeIsAdjustable',          1,                        writeable=False)
        self._InverterService.add_path('/Mode', MODE_ON,  onchangecallback = self.handle_mode_change,  writeable = True)  # default ON

        self._InverterService['/Status'] = 'ok'
        self._InverterService['/State'] = 0   # prime the state as off.  If the unit is not on it will be correct.  If is on, it will be quickly updated.

        # get the system configured CAN ID for the interface we've sellected 
        can_id_path = '/Settings/Vecan/' + CAN_INTERFACE + '/MainInterface/Nad' 
        our_can_address = vedbus.VeDbusItemImport(self.system_bus, 'com.victronenergy.settings', can_id_path).get_value()
        
        # allocate our CAN channel and register the receive packet handler
        self.nmea = NMEA2000(our_can_address)
        self.nmea.register_rx_callback (self._handle_can_frame)
                            
        # add all our desired PGNs to the filter
        for pgn, PGN_items in self.INVERTER_RX_PGN_MAP.items():
            self.nmea.add_can_filter (pgn)
        
        logger.info("Service initialization complete")

    # This method gets called when the /Mode dbus variable is changed by the
    # GUI.  We use it to change the mode that the inverter/charger is working
    # in.
    def handle_mode_change (self, path, requested_mode):
        if (requested_mode == MODE_OFF): 
            self.nmea.send_nmea_message ("CONTROL_CHARGER", 0)
            self.nmea.send_nmea_message ("CONTROL_INVERTER", 0)
        elif (requested_mode == MODE_INVERTER): 
            self.nmea.send_nmea_message ("CONTROL_CHARGER", 0)
            self.nmea.send_nmea_message ("CONTROL_INVERTER", 1)
        elif (requested_mode == MODE_CHARGER): 
            self.nmea.send_nmea_message ("CONTROL_CHARGER", 1)
            self.nmea.send_nmea_message ("CONTROL_INVERTER", 0)
        elif (requested_mode == MODE_ON):                   
            self.nmea.send_nmea_message ("CONTROL_CHARGER", 1)
            self.nmea.send_nmea_message ("CONTROL_INVERTER", 1)

        self._InverterService[path] = requested_mode
        return True                       
    
                                                  
    # Handle NMEA PGN 126983 Alarm packet
    # The NMEA alert acknowledgment package is identical to the originating
    # alert packet minus some flag bytes.  So we just copy the alert packet and
    # then append a command code.
    # The Xantrex, goes from alert state to silenced, then to acknowledged.  As
    # soon as an alarm comes fro the Xantrex, we silence and then acknowledge
    # it.
    # This is independent of the Cerbo GUI screen.  When the alert first comes
    # from the Xantrex, we return a 1 which get pushed to the GUI /Alarm/...
    # dbus entry.  The GUI does its own silencing which doesn't get pushed to
    # us.
    def alarm(self, data: bytes) -> int:
        # NMEA Alarm State values
        ALARM_STATE_NORMAL      = 1
        ALARM_STATE_ACTIVE      = 2
        ALARM_STATE_SILENCED    = 3
        ALARM_STATE_ACKED       = 4
        ALARM_STATE_WAIT_ACK    = 5

        # NMEA Alarm acknowledge values
        ALARM_ACK_ACKNOWLEDGE   = 0
        ALARM_ACK_SILENCE       = 1

        # NMEA Alarm Alert PGN 126983 useful indexes
        ALARM_IDX_ID            = 3
        ALARM_IDX_FLAGS         = 16            # flags that are removed in acknowledgement response
        ALARM_IDX_STATE         = 27
 
        alarm_state = data[ALARM_IDX_STATE]
        # parse the response packet from the incoming alert packet - This is
        # defined by NMEA PGN 126983 and PGN 126984
        alarm_response_pkt = data[:ALARM_IDX_FLAGS] + data[ALARM_IDX_FLAGS + 1: -3]
        raise_gui_alarm = 0                     # this gets sent to the GUI

        print (f"{alarm_state=}")
        if alarm_state == ALARM_STATE_ACTIVE:
            self.nmea.send_nmea_message ("ACK_ALARM", alarm_response_pkt, ALARM_ACK_SILENCE)
            raise_gui_alarm = 1                 
        elif alarm_state == ALARM_STATE_SILENCED:
            self.nmea.send_nmea_message ("ACK_ALARM", alarm_response_pkt, ALARM_ACK_ACKNOWLEDGE)
        elif alarm_state == ALARM_STATE_WAIT_ACK:
            self.nmea.send_nmea_message ("ACK_ALARM", alarm_response_pkt, ALARM_ACK_ACKNOWLEDGE)
            # we return the GUI alarm back to zero to allow for the next
            # occurance to register
            raise_gui_alarm = 0
        
        return raise_gui_alarm
    
    # This method only exist because we don't know what the NMEA codes are for
    # some Xantrex faults.  They are documented in the NMEA spec - shich I
    # don't have.  When I ask Xantrex Tech Support, they say check the spec -
    # helpful?!?
    # Anyways, this logs a warning with hopfully enough info to let us
    # determine what kind a Alert it is.  Once we know it, we can come back and
    # update the code.... eventually removing this method entirely.
    def unknown_alarm (self, data: bytes) -> int:
        alarm_id = data[3]                                      

        if alarm_id not in self.unknown_alarm_logged:
            self.unknown_alarm_logged.append (alarm_id)
            
            battery_current = self._InverterService['/Dc/0/Current']
            ac_current_out  = self._InverterService['/Ac/Out/L1/I'] 
            ac_volts_out    = self._InverterService['/Ac/Out/L1/V'] 
            ac_volts_in     = self._InverterService['/Ac/In/L1/V'] 

            logger.warning (f"Unknown Alarm ID ({alarm_id}): {battery_current=}A, {ac_volts_in=}V, {ac_volts_out=}V, {ac_current_out=}A") 

        # always return 0 because we don't want to tell the GUI stuff we don't know
        return 0
        
    # Calculate power values from voltage and current paths
    def update_derived_values(self):
        
        # Triggered whenever we receive a PGN listed in DERIVED_DGNS.
        # • Recomputes V×I power for individual measurement points
        # • Sums AC totals (input / output) and mirrors them to aliases
        # compute and publish P = V × I for inverter service
        def compute_power(dst_path: str, v_path: str, c_path: str) -> None:
            try:                              
                # Fetch the source D-Bus items
                v_item = self._InverterService[v_path]
                c_item = self._InverterService[c_path]
                
                # Skip if missing/placeholder ([], None). Zeros are OK.
                if (v_item is None or c_item is None):
                    return
                    
                # Calculate and publish power
                p = round(v_item * c_item, 1)
                self._InverterService[dst_path] = p

                logger.info(f"[{self.frame_count:06}] [DERIVED - COMPUTE POWER]  {dst_path}={p:.1f} W (V={v_item} V × I={c_item} A)")
        
            except Exception as e:
                logger.exception(f"[{self.frame_count:06}] [DERIVED FAIL]  {dst_path}={p:.1f} W (V={v_item} V × I={c_item} A)")

        # Individual power paths (DC)
        compute_power('/Dc/0/Power',   '/Dc/0/Voltage',    '/Dc/0/Current')
                                                
        
    # Process a single incoming CAN frame, decode its NMEA data, and update D-Bus paths.
    def _handle_can_frame(self, pgn, src, len, data):
        skipped_none = 0
        processed    = 0
        errors       = 0
                    
        self.frame_count += 1       
        self.isthereaframe = 1                  # indicate that there is CAN acitivity
        
        # Look up this PGN in our pre-built map.
        PGN_dict = self.INVERTER_RX_PGN_MAP.get(pgn) 

        # if this is a multiple CAN packet frame, then it needs to be assembled
        if PGN_dict['Fast_Packet']:
            self.frame_available, data = self.nmea.AssembleFastPacket(data)
        else:
            self.frame_available = True  # it's a single packet
                                                     
        # --- Decode all D-Bus values associated with this PGN and push to D-Bus  ---
        # item = (path, decoder, unit, description)
        if self.frame_available:
            PGN_items = PGN_dict['SPNs'] 
            for item in PGN_items:
                try:
                    # unpack our decoder item
                    path, decoder, unit, description = item
         
                    
                    # Safely decode data using the provided decoder function.
                    # If decoding fails, log the error and increment error counter.
                    try:
                        value = decoder(data)

                    except Exception as e:
                        self.error_count += 1
                        errors           += 1 
                        logger.error(f"[{self.frame_count:06}] [DECODE ERROR] {path}: {e} | decoder={getattr(decoder, '__name__', repr(decoder))} | data={data.hex(' ').upper()}")
                        continue

                    # If decoding failed (returned None), skip this path
                    if value is None:
                        skipped_none += 1    
                        continue

                    # special odd handling, I have not come up with a cleaner way to deal with.  
                    # if inverter reports Inverting but current is 0, force Standby ---
                    # Venus OS enum: 9 = Inverting, 1 = Standby
                    if path == '/State' and int(value) == 9:
                        # Prefer the explicit L1 voltage; fall back to the aggregate if needed
                        i_out = (self._InverterService['/Ac/Out/L1/I'] or 0)
                        # Treat no current as Standby.
                        if i_out == 0:   
                            value = 1  # Standby    
                            logger.info(f"[{self.frame_count:06}] [Inverter State OVERRIDE] Inverting→Standby; Incoming RV-C State={value} | Current out={i_out} A | Data={data.hex(' ').upper()}")
                                   
                    try:
                        self._InverterService[path] = value       # → pushes to D-Bus
                        processed += 1

                        # PGN is known and matched; value was decoded and now SENT                        
                        logger.info(f"[{self.frame_count:06}] [SENT][{self._InverterService.descriptor}] PGN=0x{pgn:05X} | path={path} | value={value} {unit} | desc=\"{description}\" | raw={data.hex(' ').upper()}")


                    except Exception as send_error:
                        logger.error(f"[{self.frame_count:06}] [DBUS SEND ERROR][{self._InverterService.descriptor}] PGN=0x{pgn:05X} | path={path} | value={value} {unit} | desc=\"{description}\" | raw={data.hex(' ').upper()} | {send_error}")
                        continue
                        
                except Exception as e:
                    self.error_count += 1
                    logger.error( f"[{self.frame_count:06}] [DECODE ERROR 2] {path}: {e} | {self.format_can_frame(pgn, data)}" )

            timestamp = time.time()  
            self._InverterService['/Mgmt/LastUpdate'] = timestamp
            logger.info(f"[{self.frame_count:06}] [FRAME SUMMARY][{self._InverterService.descriptor.upper()}] [PGN 0x{pgn:05X}] → {processed} sent, {skipped_none} null values, {errors=}")

            # After decoding known paths, calculate and send derived values
            self.update_derived_values()  
        return True     

    def start_heartbeat(self, interval=5):   # set the default to 5 s if not passed
        def _set_state(self):
            def check_path(value, default=0):
                return default if value is None else value

            inverter_service = self._InverterService

            if self.isthereaframe == 1:   # so we have a frame, which means the unit is on.
                self.isthereaframe = 0   #  reset the flag

                grid_current    = check_path( inverter_service['/Ac/In/L1/I'] )
                battery_current = check_path( inverter_service['/Dc/0/Current'] )
                ac_current_out  = check_path( inverter_service['/Ac/Out/L1/I'] )    # assume this is set only when inverting
                ac_volts_out    = check_path( inverter_service['/Ac/Out/L1/V'] )    # the volts can be out, but no current drawing 

                logger.debug(
                    f"[{self.frame_count:06}] [SET STATE] "
                    f"grid_I={grid_current:.2f}A, batt_I={battery_current:.2f}A, "
                    f"ac_out_I={ac_current_out:.2f}A, ac_out_V={ac_volts_out:.1f}V, "
                    f"inv_state={int(check_path(inverter_service['/State']))}, "
                )

            else:   # the unit is off
                inverter_service['/State'] = 0   # No frame, so show off.
  
            return True    
        
        def _publish_heartbeat():
            
            # Inner function that runs in the background thread and updates
            # the last_heartbeat timestamp and the ProcessAlive D-Bus path.
            
            try:
                # Record the current timestamp for heartbeat tracking
                self.last_heartbeat     = time.time()
                self.heartbeat_counter += 1


                # Update the D-Bus /ProcessAlive path to indicate activity
                # Update both a counter and a timestamp
                # Inverter heartbeat updates
                self._InverterService['/Mgmt/ProcessAlive']      = self.heartbeat_counter
                self._InverterService['/Mgmt/LastUpdate']        = self.last_heartbeat

                # Optionally log the heartbeat timestamp
                logger.info(f"[HEARTBEAT] {self.last_heartbeat}")
                
                # update /Status based on CAN activity
                _set_state(self)
        
            except Exception as e:
                logger.error(f"[HEARTBEAT ERROR] {e}")                    
            return True
            
        self.timeout_heartbeat = GLib.timeout_add_seconds(interval, _publish_heartbeat)
        _publish_heartbeat()          # first beat immediately

        

    # Cleanup CAN socket
    def cleanup(self):
        # === Summary Block ===
        successful_decodes = self.frame_count - self.error_count

        try:
            assert (self.error_count + successful_decodes) == self.frame_count
        except AssertionError:
            logger.warning("⚠️ Frame count mismatch in summary!")

        logger.info("=== Shutdown Summary ===")
        logger.info(f"  Total frames received      : {self.frame_count}")
        logger.info(f"  Decoded successfully       : {successful_decodes}")
        logger.info(f"  Decode errors              : {self.error_count}")
        logger.info("==========================")

        # === Cleanup Resources ===
        # Define each cleanup step as (description, callable)
        steps = [
            ("remove watch_id",              lambda: ((GLib.source_remove(self.watch_id), setattr(self, 'watch_id', None))[1] if getattr(self, 'watch_id', None) else None)),
            ("quit main loop",               lambda: self.loop.quit() if getattr(self, 'loop', None) else None),
            ("close CAN socket",             lambda: self.socket.close() if getattr(self, 'socket', None) else None),
            ("close inverter bus",           lambda: getattr(self, 'system_bus', None) and self.system_bus.close()),
            ("remove timeout_heartbeat",     lambda: GLib.source_remove(self.timeout_heartbeat) if getattr(self, 'timeout_heartbeat', None) else None),
            ("remove transmit task",         lambda: GLib.source_remove(self.transmit_task) if getattr(self, 'transmit_task', None) else None),
        ]

        # Run each step and catch its errors individually
        for desc, action in steps:
            try:
                action()
                
            except Exception as e:
                logger.error(f"[CLEAN UP] Error during '{desc}': {e}")
            
            
# === Main Application Entry Point ===
def main():
    service = None  # Ensure reference exists even if initialization fails

    def signal_handler(signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        if service and service.loop:
            service.loop.quit()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
      
        # Initialize the service with flags from CLI arguments
        service = XantrexService()

        # Set up the main GLib event loop for D-Bus
        service.loop = GLib.MainLoop()
        logger.info("Starting main loop...")

        # Start the background thread that periodically updates ProcessAlive
        #service.start_heartbeat_thread()
        service.start_heartbeat(2)


        # Enter the main loop (non-blocking events will be handled here)
        service.loop.run()

    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)
    finally:
        if service:
            service._InverterService['/Status'] = 'offline'
            service.cleanup()

        logger.info("Service stopped")


if __name__ == '__main__':
    main()