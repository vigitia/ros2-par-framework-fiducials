# Written by Claude Code
"""
Read a greyscale frame straight from a serialized sensor_msgs/Image, without rclpy.

Turning a large camera frame into a Python message costs several milliseconds of CPU per frame,
mostly copying its pixel data. Read from the serialized bytes instead, the pixels are a NumPy view
of the received buffer. Only the common case is handled: little-endian CDR with 8-bit mono or
colour pixels. Everything else returns None, and the caller converts the message the usual way.
"""

import struct

import cv2
import numpy as np

# Encapsulation header of plain little-endian CDR, the first 2 of its 4 bytes
CDR_LITTLE_ENDIAN_HEADER = b'\x00\x01'
ENCAPSULATION_HEADER_SIZE_BYTES = 4
# Encoding -> conversion to greyscale; None: already greyscale
GREYSCALE_CONVERSIONS = {
    'mono8': None,
    'bgr8': cv2.COLOR_BGR2GRAY,
    'rgb8': cv2.COLOR_RGB2GRAY,
}
CHANNEL_COUNTS = {'mono8': 1, 'bgr8': 3, 'rgb8': 3}


class _CdrReader:
    """Reads little-endian CDR fields in order; alignment counts from after the 4-byte header."""

    def __init__(self, serialized_bytes):
        self.serialized_bytes = serialized_bytes
        self.position = ENCAPSULATION_HEADER_SIZE_BYTES

    def read_uint32(self):
        self._align(4)
        (value,) = struct.unpack_from('<I', self.serialized_bytes, self.position)
        self.position += 4
        return value

    def read_int32(self):
        self._align(4)
        (value,) = struct.unpack_from('<i', self.serialized_bytes, self.position)
        self.position += 4
        return value

    def read_uint8(self):
        value = self.serialized_bytes[self.position]
        self.position += 1
        return value

    def read_string(self):
        # The length includes the terminating zero byte
        length_bytes = self.read_uint32()
        text_bytes = self._read_bytes(length_bytes)
        return text_bytes.rstrip(b'\x00').decode('utf-8')

    def read_byte_sequence_view(self):
        """Return the next uint8 sequence as a memoryview into the buffer, without copying."""
        length_bytes = self.read_uint32()
        start = self.position
        # Slicing bytes would copy them; a memoryview slice does not
        self._skip_bytes(length_bytes)
        return memoryview(self.serialized_bytes)[start:start + length_bytes]

    def _align(self, alignment_bytes):
        offset_in_body = self.position - ENCAPSULATION_HEADER_SIZE_BYTES
        padding_bytes = -offset_in_body % alignment_bytes
        self.position += padding_bytes

    def _read_bytes(self, length_bytes):
        start = self.position
        self._skip_bytes(length_bytes)
        return self.serialized_bytes[start:self.position]

    def _skip_bytes(self, length_bytes):
        end = self.position + length_bytes
        if end > len(self.serialized_bytes):
            raise ValueError('serialized image ends before its fields do')
        self.position = end


def read_serialized_image_mono(serialized_bytes):
    """
    Return (stamp_sec, stamp_nanosec, frame_id, frame_mono) from a serialized Image, or None.

    frame_mono is a height x width uint8 array; for mono8 it shares memory with serialized_bytes
    and is read-only. None means this message needs the usual conversion: another CDR
    representation, another encoding, or a size that does not match the pixel data.
    """
    if serialized_bytes[:2] != CDR_LITTLE_ENDIAN_HEADER:
        return None
    reader = _CdrReader(serialized_bytes)
    try:
        stamp_sec = reader.read_int32()
        stamp_nanosec = reader.read_uint32()
        frame_id = reader.read_string()
        height_px = reader.read_uint32()
        width_px = reader.read_uint32()
        encoding = reader.read_string()
        reader.read_uint8()  # is_bigendian: irrelevant for 8-bit pixels
        step_bytes = reader.read_uint32()
        pixel_data = reader.read_byte_sequence_view()
    except (ValueError, struct.error, IndexError, UnicodeDecodeError):
        return None

    if encoding not in GREYSCALE_CONVERSIONS:
        return None
    row_size_bytes = width_px * CHANNEL_COUNTS[encoding]
    # A sender may pad rows (step > row size), but the data must hold every row
    if height_px == 0 or step_bytes < row_size_bytes or len(pixel_data) < step_bytes * height_px:
        return None

    rows = np.frombuffer(pixel_data, np.uint8, count=step_bytes * height_px)
    rows = rows.reshape(height_px, step_bytes)[:, :row_size_bytes]
    conversion = GREYSCALE_CONVERSIONS[encoding]
    if conversion is None:
        frame_mono = rows
    else:
        frame_colour = rows.reshape(height_px, width_px, CHANNEL_COUNTS[encoding])
        frame_mono = cv2.cvtColor(frame_colour, conversion)
    return stamp_sec, stamp_nanosec, frame_id, frame_mono
