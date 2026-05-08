#!/usr/bin/env python3
"""
EPOS2 Profile Velocity Mode driver for Raspberry Pi.

Drives a Maxon EC motor through a Maxon EPOS2 controller over USB,
using Maxon's EposCmd Linux shared library (libEposCmd.so).

Hardware assumptions:
  - Pi <-> EPOS2 over USB
  - EC motor wired and configured (Hall sensors / encoder set up in EPOS Studio first)
  - NodeID set on EPOS2 (default 1)

Software prerequisites:
  1. Install Maxon EPOS Command Library for Linux ARM.
     The .deb / tarball ships libEposCmd.so; copy it to /usr/local/lib
     and run `sudo ldconfig`.
  2. Install the udev rule that ships with the library so the 'pi'
     user can access the EPOS2 USB device without sudo.
  3. Run `pip3 install` is NOT needed - this only uses ctypes.

Run motor tuning, current limits, and encoder configuration in
EPOS Studio on a Windows machine before driving it from the Pi.
This script assumes the controller already holds a valid config.
"""

import ctypes
import sys
import time
from ctypes import byref, c_int, c_uint, c_ushort, c_void_p

# ---------- Connection settings ----------
EPOS_LIB        = "libEposCmd.so"
DEVICE_NAME     = b"EPOS2"
PROTOCOL_STACK  = b"MAXON SERIAL V2"
INTERFACE       = b"USB"
PORT_NAME       = b"USB0"
BAUDRATE        = 1_000_000      # 1 Mbit/s on USB
TIMEOUT_MS      = 500
NODE_ID         = 1              # match the DIP-switch / EPOS Studio setting

# ---------- Motion settings ----------
PROFILE_ACCEL   = 5_000          # rpm/s
PROFILE_DECEL   = 5_000          # rpm/s


class EPOS2:
    """Thin ctypes wrapper around the EPOS Command Library."""

    def __init__(self):
        self.lib = ctypes.cdll.LoadLibrary(EPOS_LIB)
        self.handle = None
        self.err = c_uint(0)
        self.node = c_ushort(NODE_ID)
        self._configure_signatures()

    def _configure_signatures(self):
        # Only the return type for OpenDevice really matters - the rest
        # default to int and we pass the right argument types explicitly.
        self.lib.VCS_OpenDevice.restype = c_void_p

    def _raise(self, action):
        buf = ctypes.create_string_buffer(256)
        self.lib.VCS_GetErrorInfo(self.err, buf, 256)
        raise RuntimeError(f"{action} failed (0x{self.err.value:08X}): "
                           f"{buf.value.decode(errors='replace')}")

    def _check(self, ret, action):
        if ret == 0:
            self._raise(action)

    # ----- lifecycle -----
    def open(self):
        self.handle = self.lib.VCS_OpenDevice(
            DEVICE_NAME, PROTOCOL_STACK, INTERFACE, PORT_NAME, byref(self.err)
        )
        if not self.handle:
            self._raise("OpenDevice")

        ret = self.lib.VCS_SetProtocolStackSettings(
            c_void_p(self.handle), c_uint(BAUDRATE),
            c_uint(TIMEOUT_MS), byref(self.err)
        )
        self._check(ret, "SetProtocolStackSettings")

        # Clear any latched fault so we can enable cleanly.
        self.lib.VCS_ClearFault(c_void_p(self.handle), self.node, byref(self.err))

    def close(self):
        if self.handle:
            self.lib.VCS_CloseDevice(c_void_p(self.handle), byref(self.err))
            self.handle = None

    # ----- state -----
    def enable(self):
        ret = self.lib.VCS_SetEnableState(
            c_void_p(self.handle), self.node, byref(self.err)
        )
        self._check(ret, "SetEnableState")

    def disable(self):
        if self.handle:
            self.lib.VCS_SetDisableState(
                c_void_p(self.handle), self.node, byref(self.err)
            )

    # ----- profile velocity mode -----
    def activate_velocity_mode(self, accel=PROFILE_ACCEL, decel=PROFILE_DECEL):
        ret = self.lib.VCS_ActivateProfileVelocityMode(
            c_void_p(self.handle), self.node, byref(self.err)
        )
        self._check(ret, "ActivateProfileVelocityMode")

        ret = self.lib.VCS_SetVelocityProfile(
            c_void_p(self.handle), self.node,
            c_uint(int(accel)), c_uint(int(decel)),
            byref(self.err)
        )
        self._check(ret, "SetVelocityProfile")

    def set_velocity(self, rpm):
        """Signed 32-bit target velocity in motor rpm."""
        ret = self.lib.VCS_MoveWithVelocity(
            c_void_p(self.handle), self.node,
            c_int(int(rpm)), byref(self.err)
        )
        self._check(ret, "MoveWithVelocity")

    def halt(self):
        self.lib.VCS_HaltVelocityMovement(
            c_void_p(self.handle), self.node, byref(self.err)
        )

    def get_velocity(self):
        v = c_int(0)
        self.lib.VCS_GetVelocityIs(
            c_void_p(self.handle), self.node, byref(v), byref(self.err)
        )
        return v.value


def main():
    motor = EPOS2()
    try:
        motor.open()
        print("EPOS2 opened.")

        motor.activate_velocity_mode()
        motor.enable()
        print("Profile Velocity Mode active. Running setpoint sequence.")

        # Example sequence: ramp up, hold, reverse, stop.
        # Each setpoint runs for `dwell_s` seconds while we poll actual speed.
        sequence = [(1000, 2.0),
                    (2000, 2.0),
                    (   0, 1.0),
                    (-1500, 2.0),
                    (   0, 1.0)]

        for target_rpm, dwell_s in sequence:
            print(f"  target = {target_rpm:+5d} rpm")
            motor.set_velocity(target_rpm)
            t_end = time.monotonic() + dwell_s
            while time.monotonic() < t_end:
                print(f"    actual = {motor.get_velocity():+5d} rpm")
                time.sleep(0.1)

        motor.halt()

    except KeyboardInterrupt:
        print("\nInterrupted - halting motor.")
        try:
            motor.halt()
        except RuntimeError:
            pass
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
    finally:
        motor.disable()
        motor.close()
        print("EPOS2 closed.")


if __name__ == "__main__":
    main()