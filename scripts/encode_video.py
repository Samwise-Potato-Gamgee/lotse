"""Turns a raw Isaac Sim recording plus the run log into a labelled H.264 mp4
with the index at the front, a sped-up clip, a GIF that shows a robot waiting
for another, and three stills. Each composed frame has a header strip, the
scaled warehouse scene and a legend panel that never overlap. The overlay is
drawn here, frame by frame, only from the run log, the camera framing and the
map. Runs on the host because the simulator image cannot encode H.264.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image

CONTAINER_BOXES = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"dinf", b"edts"}

FRAME_COLOUR_BGR = (18, 18, 22)
HEADER_HEIGHT = 48
LEGEND_WIDTH = 320
SCENE_MARGIN_PX = 10

RENDERED_ROBOT_COLOURS_BGR = [(148, 146, 228), (237, 210, 145), (198, 206, 134),
                              (237, 179, 230)]
RENDERED_PICKUP_COLOUR_BGR = (188, 229, 79)
RENDERED_DROPOFF_COLOUR_BGR = (51, 214, 238)
RENDERED_PARKING_COLOUR_BGR = (238, 199, 155)

WAIT_OCCUPIED_COLOUR_BGR = (0, 0, 0)
WAIT_RIGHT_OF_WAY_COLOUR_BGR = (255, 0, 0)
WAIT_COLOURS_BGR = {"cell_occupied": WAIT_OCCUPIED_COLOUR_BGR,
                    "pass_order": WAIT_RIGHT_OF_WAY_COLOUR_BGR}
WAIT_LABELS = {"cell_occupied": ("waiting: next cell", "still occupied"),
               "pass_order": ("waiting: right of way",)}
BAR_COLOUR_BGR = (25, 25, 25)
PANEL_COLOUR_BGR = (228, 228, 231)
PANEL_TEXT_BGR = (30, 30, 35)
PANEL_EDGE_BGR = (110, 110, 118)

HIGH_PROFILES = {100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135}


def walk_boxes(data, start, end):
    """Yields (type, start, size) for the boxes between start and end.
    @param data the mp4 bytes
    @param start the first byte to read
    @param end the last byte to read
    """
    pos = start
    while pos + 8 <= end:
        size = int.from_bytes(data[pos:pos + 4], "big")
        kind = data[pos + 4:pos + 8]
        if size == 1:
            size = int.from_bytes(data[pos + 8:pos + 16], "big")
        elif size == 0:
            size = end - pos
        if size < 8 or pos + size > end:
            break
        yield kind, pos, size
        if kind in CONTAINER_BOXES:
            yield from walk_boxes(data, pos + 8, pos + size)
        pos += size


def top_level_box(data, kind):
    """Returns the (start, size) of a top-level box, or None.
    @param data the mp4 bytes
    @param kind the box type, for example b"moov"
    """
    pos = 0
    end = len(data)
    while pos + 8 <= end:
        size = int.from_bytes(data[pos:pos + 4], "big")
        box_kind = data[pos + 4:pos + 8]
        if size == 1:
            size = int.from_bytes(data[pos + 8:pos + 16], "big")
        elif size == 0:
            size = end - pos
        if size < 8 or pos + size > end:
            break
        if box_kind == kind:
            return pos, size
        pos += size
    return None


def patch_chunk_offsets(moov, delta):
    """Adds delta to every stco and co64 chunk offset inside a moov buffer.
    @param moov the moov box bytes
    @param delta the byte shift applied to the media data
    """
    result = bytearray(moov)
    for kind, start, _ in list(walk_boxes(moov, 0, len(moov))):
        if kind not in (b"stco", b"co64"):
            continue
        count = int.from_bytes(result[start + 12:start + 16], "big")
        width = 4 if kind == b"stco" else 8
        base = start + 16
        for entry in range(count):
            at = base + entry * width
            value = int.from_bytes(result[at:at + width], "big") + delta
            result[at:at + width] = value.to_bytes(width, "big")
    return bytes(result)


def faststart(data):
    """Moves the moov box in front of the mdat box and fixes chunk offsets.
    @param data the mp4 bytes
    """
    moov = top_level_box(data, b"moov")
    mdat = top_level_box(data, b"mdat")
    if moov is None or mdat is None or moov[0] < mdat[0]:
        return data
    moov_start, moov_size = moov
    mdat_start, mdat_size = mdat
    prefix = data[:mdat_start]
    media = data[mdat_start:mdat_start + mdat_size]
    rest = data[mdat_start + mdat_size:]
    offset = moov_start - (mdat_start + mdat_size)
    rest = rest[:offset] + rest[offset + moov_size:]
    moved = patch_chunk_offsets(data[moov_start:moov_start + moov_size], moov_size)
    return prefix + moved + media + rest


def rewrite_faststart(path):
    """Rewrites an mp4 in place with its moov box in front of the media data.
    @param path the mp4 file to rewrite
    """
    with open(path, "rb") as handle:
        data = handle.read()
    with open(path, "wb") as handle:
        handle.write(faststart(data))


class BitReader:
    """Reads Exp-Golomb and fixed-width fields from an RBSP byte string."""

    def __init__(self, data):
        """Stores the unescaped RBSP byte string to read.
        @param data the RBSP bytes
        """
        self.data = data
        self.pos = 0

    def bits(self, count):
        """Reads count bits as an unsigned integer.
        @param count how many bits to read
        """
        value = 0
        for _ in range(count):
            byte = self.data[self.pos >> 3]
            value = (value << 1) | ((byte >> (7 - (self.pos & 7))) & 1)
            self.pos += 1
        return value

    def ue(self):
        """Reads one unsigned Exp-Golomb value."""
        zeros = 0
        while self.bits(1) == 0:
            zeros += 1
        return ((1 << zeros) - 1) + (self.bits(zeros) if zeros else 0)

    def se(self):
        """Reads one signed Exp-Golomb value."""
        value = self.ue()
        return (value + 1) // 2 if value % 2 else -(value // 2)


def remove_emulation(data):
    """Removes the 0x000003 escaping used inside an RBSP byte string.
    @param data the escaped bytes
    """
    result = bytearray()
    index = 0
    while index < len(data):
        if data[index:index + 3] == b"\x00\x00\x03":
            result += b"\x00\x00"
            index += 3
        else:
            result.append(data[index])
            index += 1
    return bytes(result)


def skip_scaling_list(reader, size):
    """Skips one scaling list in an SPS.
    @param reader the bit reader positioned at the list
    @param size the number of coefficients, 16 or 64
    """
    last = 8
    nxt = 8
    for _ in range(size):
        if nxt != 0:
            nxt = (last + reader.se() + 256) % 256
        last = nxt


def sps_chroma_format(sps):
    """Returns chroma_format_idc from one SPS NAL unit, or None when unreadable.
    @param sps the SPS NAL unit including its header byte
    """
    reader = BitReader(remove_emulation(sps[1:]))
    profile = reader.bits(8)
    reader.bits(8)
    reader.bits(8)
    reader.ue()
    chroma = 1
    if profile in HIGH_PROFILES:
        chroma = reader.ue()
        if chroma == 3:
            reader.bits(1)
        reader.ue()
        reader.ue()
        reader.bits(1)
        if reader.bits(1):
            lists = 8 if chroma != 3 else 12
            for index in range(lists):
                if reader.bits(1):
                    skip_scaling_list(reader, 16 if index < 6 else 64)
    return chroma


def chroma_format(path):
    """Returns the AVC chroma format of an mp4 as text, or "unknown".
    @param path the mp4 file to read
    """
    with open(path, "rb") as handle:
        data = handle.read()
    marker = data.find(b"avcC")
    if marker < 0:
        return "unknown"
    at = marker + 4 + 1 + 3 + 1
    count = data[at] & 0x1F
    at += 1
    if count < 1:
        return "unknown"
    length = int.from_bytes(data[at:at + 2], "big")
    at += 2
    value = sps_chroma_format(data[at:at + length])
    if value == 1:
        return "yuv420p"
    if value == 2:
        return "yuv422p"
    if value == 3:
        return "yuv444p"
    return "unknown"


def load_camera(path):
    """Loads the top-down camera framing written by the scene.
    @param path the camera.json file
    """
    with open(path) as handle:
        return json.load(handle)


def load_run_log(path):
    """Loads the run log into a list of frames with absolute simulated times.
    @param path the run_log.jsonl file
    """
    log = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                log.append(json.loads(line))
    log.sort(key=lambda entry: entry["t_abs"])
    return log


def load_frame_times(path):
    """Loads the simulated time of every recorded frame.
    @param path the frames.jsonl file
    """
    times = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                times.append(json.loads(line)["t"])
    return times


def load_map(path):
    """Reads a Lotse map file into its stations, parking cells and grid size.
    @param path the map file
    """
    with open(path) as handle:
        lines = [line.rstrip("\n") for line in handle]
    width = int(next(line for line in lines if line.startswith("width")).split()[1])
    height = int(next(line for line in lines if line.startswith("height")).split()[1])
    stations = []
    if "stations" in lines:
        start = lines.index("stations") + 1
        end = lines.index("parking") if "parking" in lines else len(lines)
        for line in lines[start:end]:
            parts = line.split()
            if len(parts) == 3:
                stations.append((parts[0], int(parts[1]), int(parts[2])))
    parking = []
    if "parking" in lines:
        for line in lines[lines.index("parking") + 1:]:
            parts = line.split()
            if len(parts) == 2:
                parking.append((int(parts[0]), int(parts[1])))
    return {"stations": stations, "parking": parking, "width": width, "height": height}


def project(camera, x, y):
    """Projects a world (x, y) to a source-image pixel with the top-down camera.
    @param camera the camera framing dictionary
    @param x the world x in metres
    @param y the world y in metres
    """
    left = camera["centre_x"] - camera["half_width_m"]
    top = camera["centre_y"] + camera["half_height_m"]
    u = (x - left) / (2.0 * camera["half_width_m"]) * camera["image_width"]
    v = (top - y) / (2.0 * camera["half_height_m"]) * camera["image_height"]
    return int(round(u)), int(round(v))


def scene_layout(camera, map_width, map_height):
    """Returns the crop and scale that fit the warehouse into the scene area.
    @param camera the camera framing dictionary
    @param map_width the warehouse width in cells
    @param map_height the warehouse height in cells
    """
    left, bottom = project(camera, 0.0, 0.0)
    right, top = project(camera, map_width, map_height)
    crop_x0 = max(0, left - SCENE_MARGIN_PX)
    crop_y0 = max(0, top - SCENE_MARGIN_PX)
    crop_x1 = min(camera["image_width"], right + SCENE_MARGIN_PX)
    crop_y1 = min(camera["image_height"], bottom + SCENE_MARGIN_PX)
    crop_w = crop_x1 - crop_x0
    crop_h = crop_y1 - crop_y0
    area_w = camera["image_width"] - LEGEND_WIDTH
    area_h = camera["image_height"] - HEADER_HEIGHT
    scale = min(area_w / crop_w, area_h / crop_h)
    off_x = (area_w - crop_w * scale) / 2.0
    off_y = HEADER_HEIGHT + (area_h - crop_h * scale) / 2.0
    return {"crop_x0": crop_x0, "crop_y0": crop_y0, "crop_w": crop_w, "crop_h": crop_h,
            "scale": scale, "off_x": off_x, "off_y": off_y}


def to_frame(layout, camera, x, y):
    """Projects a world position into the composed frame.
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param x the world x in metres
    @param y the world y in metres
    """
    sx, sy = project(camera, x, y)
    u = layout["off_x"] + (sx - layout["crop_x0"]) * layout["scale"]
    v = layout["off_y"] + (sy - layout["crop_y0"]) * layout["scale"]
    return int(round(u)), int(round(v))


def compose_frame(raw, layout):
    """Crops the empty margin and scales the warehouse into the scene area.
    @param raw the raw scene frame
    @param layout the scene layout
    """
    x0 = int(round(layout["crop_x0"]))
    y0 = int(round(layout["crop_y0"]))
    x1 = int(round(layout["crop_x0"] + layout["crop_w"]))
    y1 = int(round(layout["crop_y0"] + layout["crop_h"]))
    target_w = int(round(layout["crop_w"] * layout["scale"]))
    target_h = int(round(layout["crop_h"] * layout["scale"]))
    scene = cv2.resize(raw[y0:y1, x0:x1], (target_w, target_h), interpolation=cv2.INTER_AREA)
    canvas = np.full((raw.shape[0], raw.shape[1], 3), FRAME_COLOUR_BGR, np.uint8)
    off_x = int(round(layout["off_x"]))
    off_y = int(round(layout["off_y"]))
    canvas[off_y:off_y + target_h, off_x:off_x + target_w] = scene
    return canvas


def state_at(log, time):
    """Returns the run-log frame whose absolute simulated time is closest.
    @param log the sorted run log
    @param time the wanted simulated time
    """
    if not log:
        return None
    best = log[0]
    best_gap = abs(log[0]["t_abs"] - time)
    for entry in log:
        gap = abs(entry["t_abs"] - time)
        if gap < best_gap:
            best = entry
            best_gap = gap
    return best


def draw_label(frame, text, origin, scale, colour):
    """Draws text with a dark outline so it stays readable on any floor.
    @param frame the image to draw on
    @param text the text
    @param origin the (x, y) of the text baseline's left edge
    @param scale the font scale
    @param colour the text colour in BGR
    """
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)


def draw_top_bar(frame, state, total_orders, speed_text):
    """Draws simulated time, orders done and the playback speed along the top.
    @param frame the image to draw on
    @param state the run-log frame
    @param total_orders how many orders the run releases
    @param speed_text the playback speed text
    """
    width = frame.shape[1]
    panel = frame.copy()
    cv2.rectangle(panel, (0, 0), (width, HEADER_HEIGHT), BAR_COLOUR_BGR, -1)
    cv2.addWeighted(panel, 0.80, frame, 0.20, 0, frame)
    text = ("simulated time %.1f s     orders %d / %d     playback %s"
            % (state["t"], state["done"], total_orders, speed_text))
    draw_label(frame, text, (14, 32), 0.72, (255, 255, 255))


def draw_stations(frame, layout, camera, stations):
    """Draws every named station on its cell, clamped inside the frame.
    @param frame the image to draw on
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param stations the (name, x, y) stations
    """
    height, width = frame.shape[:2]
    for name, x, y in stations:
        u, v = to_frame(layout, camera, x + 0.5, y + 0.5)
        origin_x = min(max(u - 30, 2), width - LEGEND_WIDTH - 70)
        origin_y = min(max(v + 34, HEADER_HEIGHT + 14), height - 6)
        draw_label(frame, name, (origin_x, origin_y), 0.45, (255, 255, 255))


def draw_robots(frame, layout, camera, state):
    """Draws each robot's number and a coloured ring for either wait reason.
    @param frame the image to draw on
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param state the run-log frame
    """
    for robot in state["robots"]:
        u, v = to_frame(layout, camera, robot["x"], robot["y"])
        colour = WAIT_COLOURS_BGR.get(robot["wait"])
        if colour is not None:
            cv2.circle(frame, (u, v), 23, (255, 255, 255), 7, cv2.LINE_AA)
            cv2.circle(frame, (u, v), 23, colour, 4, cv2.LINE_AA)
        label_x = min(max(u + 13, 2), frame.shape[1] - LEGEND_WIDTH - 30)
        draw_label(frame, "R%d" % robot["id"], (label_x, v - 15), 0.55, (255, 255, 255))


def robot_task_text(robot):
    """Returns one robot's task in words for the legend.
    @param robot the run-log robot entry
    """
    if robot["phase"] == "parked":
        return "parked at %s" % robot["station"]
    return "to %s %s" % (robot["phase"], robot["station"])


def draw_swatch(frame, left, row, colour):
    """Draws a legend colour swatch with a dark border so pastel colours read.
    @param frame the image to draw on
    @param left the panel left edge
    @param row the swatch text baseline
    @param colour the swatch colour in BGR
    """
    cv2.rectangle(frame, (left + 10, row - 12), (left + 28, row + 2), PANEL_EDGE_BGR, -1)
    cv2.rectangle(frame, (left + 11, row - 11), (left + 27, row + 1), colour, -1)


def draw_legend(frame, layout, state, map_data):
    """Draws the colour legend and the two waiting markers on the right.
    @param frame the image to draw on
    @param layout the scene layout
    @param state the run-log frame
    @param map_data the map dictionary
    """
    height, width = frame.shape[:2]
    left = width - LEGEND_WIDTH
    panel = frame.copy()
    cv2.rectangle(panel, (left, HEADER_HEIGHT), (width, height), PANEL_COLOUR_BGR, -1)
    cv2.addWeighted(panel, 0.88, frame, 0.12, 0, frame)
    cv2.line(frame, (left, HEADER_HEIGHT), (left, height), PANEL_EDGE_BGR, 1)
    row = HEADER_HEIGHT + 28
    draw_label(frame, "Lotse fleet", (left + 10, row), 0.52, PANEL_TEXT_BGR)
    row += 26
    for robot in state["robots"]:
        colour = RENDERED_ROBOT_COLOURS_BGR[robot["id"] % len(RENDERED_ROBOT_COLOURS_BGR)]
        draw_swatch(frame, left, row, colour)
        draw_label(frame, "R%d" % robot["id"], (left + 36, row), 0.48, PANEL_TEXT_BGR)
        row += 20
        draw_label(frame, robot_task_text(robot), (left + 36, row), 0.42, PANEL_TEXT_BGR)
        row += 20
        for line in WAIT_LABELS.get(robot["wait"], ()):
            draw_label(frame, line, (left + 36, row), 0.40, WAIT_COLOURS_BGR[robot["wait"]])
            row += 17
        row += 12
    row += 4
    for name, colour in (("pickup", RENDERED_PICKUP_COLOUR_BGR),
                         ("dropoff", RENDERED_DROPOFF_COLOUR_BGR),
                         ("parking", RENDERED_PARKING_COLOUR_BGR)):
        draw_swatch(frame, left, row, colour)
        draw_label(frame, name, (left + 36, row), 0.44, PANEL_TEXT_BGR)
        row += 24
    row += 8
    cv2.circle(frame, (left + 19, row - 5), 9, PANEL_EDGE_BGR, 5, cv2.LINE_AA)
    cv2.circle(frame, (left + 19, row - 5), 9, WAIT_OCCUPIED_COLOUR_BGR, 3, cv2.LINE_AA)
    for line in WAIT_LABELS["cell_occupied"]:
        draw_label(frame, line, (left + 36, row), 0.40, WAIT_OCCUPIED_COLOUR_BGR)
        row += 17
    row += 8
    cv2.circle(frame, (left + 19, row - 5), 9, PANEL_EDGE_BGR, 5, cv2.LINE_AA)
    cv2.circle(frame, (left + 19, row - 5), 9, WAIT_RIGHT_OF_WAY_COLOUR_BGR, 3, cv2.LINE_AA)
    for line in WAIT_LABELS["pass_order"]:
        draw_label(frame, line, (left + 36, row), 0.40, WAIT_RIGHT_OF_WAY_COLOUR_BGR)
        row += 17
    draw_label(frame, "%d stations" % len(map_data["stations"]), (left + 10, height - 14),
               0.40, PANEL_EDGE_BGR)


def draw_overlay(frame, state, layout, camera, map_data, total_orders, speed_text):
    """Draws the whole overlay for one composed frame from the run-log state.
    @param frame the image to draw on
    @param state the run-log frame
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param map_data the map dictionary
    @param total_orders how many orders the run releases
    @param speed_text the playback speed text
    """
    if state is None:
        return
    draw_stations(frame, layout, camera, map_data["stations"])
    draw_robots(frame, layout, camera, state)
    draw_top_bar(frame, state, total_orders, speed_text)
    draw_legend(frame, layout, state, map_data)


def open_writer(path, fps, width, height):
    """Creates an H.264 mp4 writer and returns it.
    @param path the output file
    @param fps the output frame rate
    @param width the output width in pixels
    @param height the output height in pixels
    """
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("the host OpenCV cannot encode H.264")
    return writer


def render_video(raw_path, out_path, fps, camera, layout, run_log, map_data, frame_times,
                 total_orders, step, scale, speed_text):
    """Composes the scene, draws the overlay and re-encodes, optionally sped up.
    @param raw_path the raw recording
    @param out_path the H.264 mp4 to write
    @param fps the output frame rate
    @param camera the camera framing dictionary
    @param layout the scene layout
    @param run_log the run log
    @param map_data the map dictionary
    @param frame_times the simulated time of every raw frame
    @param total_orders how many orders the run releases
    @param step keep every step-th frame, so the clip plays step times faster
    @param scale the output size as a fraction of the composed size
    @param speed_text the playback speed shown in the overlay
    """
    source = cv2.VideoCapture(raw_path)
    if not source.isOpened():
        raise RuntimeError("cannot open " + raw_path)
    width = int(camera["image_width"] * scale)
    height = int(camera["image_height"] * scale)
    writer = open_writer(out_path, fps, width, height)
    index = 0
    kept = 0
    while True:
        ok, raw = source.read()
        if not ok:
            break
        if index % step == 0:
            frame = compose_frame(raw, layout)
            time = frame_times[index] if index < len(frame_times) else None
            draw_overlay(frame, state_at(run_log, time) if time is not None else None,
                         layout, camera, map_data, total_orders, speed_text)
            if scale != 1.0:
                frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            writer.write(frame)
            kept += 1
        index += 1
    source.release()
    writer.release()
    rewrite_faststart(out_path)
    return kept, width, height


def collect_waits(run_log):
    """Returns every continuous wait interval with its kind, robot and times.
    @param run_log the run log
    """
    robot_count = len(run_log[0]["robots"]) if run_log else 0
    current = [None] * robot_count
    started = [0.0] * robot_count
    waits = []
    for entry in run_log:
        for index, robot in enumerate(entry["robots"]):
            kind = robot["wait"] if robot["wait"] in WAIT_COLOURS_BGR else "none"
            if current[index] is None:
                current[index] = kind
                started[index] = entry["t_abs"]
            elif kind != current[index]:
                if current[index] != "none":
                    waits.append({"kind": current[index], "robot": index,
                                  "start": started[index], "end": entry["t_abs"]})
                current[index] = kind
                started[index] = entry["t_abs"]
    last = run_log[-1]["t_abs"] if run_log else 0.0
    for index in range(robot_count):
        if current[index] not in (None, "none"):
            waits.append({"kind": current[index], "robot": index,
                          "start": started[index], "end": last})
    return waits


def find_wait_window(run_log, waits, length):
    """Returns a window of the wanted length that contains the longest wait whole.
    @param run_log the run log
    @param waits the wait intervals from collect_waits
    @param length how many simulated seconds the window should cover
    """
    if not waits:
        return None
    longest = max(waits, key=lambda wait: wait["end"] - wait["start"])
    run_end = run_log[-1]["t_abs"]
    middle = (longest["start"] + longest["end"]) / 2.0
    start = min(max(0.0, middle - length / 2.0), max(0.0, run_end - length))
    return start, start + length, longest["kind"], longest


def wait_summary(waits):
    """Returns the count, total duration and longest duration of each wait kind.
    @param waits the wait intervals from collect_waits
    """
    summary = {}
    for kind in WAIT_COLOURS_BGR:
        kind_waits = [wait for wait in waits if wait["kind"] == kind]
        durations = [wait["end"] - wait["start"] for wait in kind_waits]
        summary[kind] = {"count": len(kind_waits), "total": sum(durations),
                         "longest": max(durations, default=0.0)}
    return summary


def render_gif(raw_path, out_path, camera, layout, run_log, map_data, frame_times, total_orders,
               window, fps, width, colours, max_bytes):
    """Writes a labelled GIF of one waiting window, shrinking it until it fits.
    @param raw_path the raw recording
    @param out_path the GIF to write
    @param camera the camera framing dictionary
    @param layout the scene layout
    @param run_log the run log
    @param map_data the map dictionary
    @param frame_times the simulated time of every raw frame
    @param total_orders how many orders the run releases
    @param window the (start, end, kind, wait) simulated-time window
    @param fps the GIF frame rate
    @param width the GIF width in pixels
    @param colours how many GIF palette colours
    @param max_bytes the size limit
    """
    source = cv2.VideoCapture(raw_path)
    if not source.isOpened():
        raise RuntimeError("cannot open " + raw_path)
    raw_fps = source.get(cv2.CAP_PROP_FPS) or 10.0
    step = max(1, int(round(raw_fps / fps)))
    scale = width / source.get(cv2.CAP_PROP_FRAME_WIDTH)
    start, end = window[0], window[1]
    frames = []
    index = 0
    while True:
        ok, raw = source.read()
        if not ok:
            break
        time = frame_times[index] if index < len(frame_times) else None
        if time is not None and start <= time <= end and index % step == 0:
            frame = compose_frame(raw, layout)
            draw_overlay(frame, state_at(run_log, time), layout, camera, map_data,
                         total_orders, "1x")
            frames.append(cv2.resize(frame, (width, int(frame.shape[0] * scale)),
                                     interpolation=cv2.INTER_AREA))
        index += 1
    source.release()
    if not frames:
        return 0
    duration = int(round(1000.0 / fps))
    for palette in (colours, 32, 24, 16):
        images = [Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert(
            "P", palette=Image.ADAPTIVE, colors=palette) for frame in frames]
        images[0].save(out_path, save_all=True, append_images=images[1:], duration=duration,
                       loop=0, optimize=True)
        if os.path.getsize(out_path) <= max_bytes:
            break
    return len(frames)


def gif_probe(path):
    """Fully decodes a GIF and returns its size, frame count, duration and bytes.
    @param path the GIF to decode
    """
    image = Image.open(path)
    frames = 0
    duration_ms = 0
    try:
        while True:
            frames += 1
            duration_ms += image.info.get("duration", 0)
            image.seek(image.tell() + 1)
    except EOFError:
        pass
    return {"width": image.size[0], "height": image.size[1], "frames": frames,
            "duration": duration_ms / 1000.0, "bytes": os.path.getsize(path)}


def probe(path):
    """Fully decodes a video and returns its size, frame and pixel statistics.
    @param path the video to decode
    """
    capture = cv2.VideoCapture(path)
    if not capture.isOpened():
        return None
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    frames = 0
    bad = 0
    spread = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames += 1
        if frame is None or frame.size == 0:
            bad += 1
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        spread.append((round(float(gray.mean()), 2), round(float(gray.std()), 2),
                       int(gray.min()), int(gray.max())))
    capture.release()
    return {"width": width, "height": height, "fps": fps, "frames": frames,
            "bad": bad, "spread": spread}


def overlay_pixels(frame):
    """Counts legend pixels close to an overlay robot colour.
    @param frame the image to test
    """
    height, width = frame.shape[:2]
    column = frame[HEADER_HEIGHT:height, width - LEGEND_WIDTH:width].astype(int)
    total = 0
    for colour in RENDERED_ROBOT_COLOURS_BGR:
        distance = np.sqrt(np.sum((column - np.array(colour, dtype=int)) ** 2, axis=2))
        total += int((distance < 40).sum())
    return total


def still(path, fraction, out_path):
    """Saves one frame at a fraction through a video as a PNG and returns stats.
    @param path the video to read
    @param fraction where to take the frame, 0..1
    @param out_path the PNG to write
    """
    capture = cv2.VideoCapture(path)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    index = int(total * fraction)
    capture.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, frame = capture.read()
    capture.release()
    if not ok or frame is None:
        return None
    cv2.imwrite(out_path, frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {"mean": round(float(gray.mean()), 2), "std": round(float(gray.std()), 2),
            "min": int(gray.min()), "max": int(gray.max()), "path": out_path,
            "gray": gray, "overlay": overlay_pixels(frame)}


def detect_disc(frame, layout, camera, x, y, colour_bgr):
    """Returns the centroid of a robot disc near a planned robot position.
    @param frame the image to search
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param x the world x in metres
    @param y the world y in metres
    @param colour_bgr the robot's rendered disc colour in BGR
    """
    height, width = frame.shape[:2]
    u, v = to_frame(layout, camera, x, y)
    right_edge = width - LEGEND_WIDTH
    window = 40
    x0 = max(0, u - window)
    x1 = min(right_edge, u + window)
    y0 = max(HEADER_HEIGHT, v - window)
    y1 = min(height - 1, v + window)
    if x1 <= x0 or y1 <= y0:
        return None
    hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    reference = cv2.cvtColor(np.uint8([[colour_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
    hue = hsv[:, :, 0].astype(int)
    gap = np.abs(hue - int(reference[0]))
    gap = np.minimum(gap, 180 - gap)
    mask = (gap <= 18) & (hsv[:, :, 1] > 60) & (hsv[:, :, 2] > 70)
    if int(mask.sum()) < 20:
        return None
    rows, columns = np.nonzero(mask)
    return (x0 + float(columns.mean()), y0 + float(rows.mean()))


def marker_pixels(frame, layout, camera, x, y, colour):
    """Counts pixels near a marker colour within 44 px of a world position.
    @param frame the image to count in
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param x the world x in metres
    @param y the world y in metres
    @param colour the marker colour in BGR
    """
    height, width = frame.shape[:2]
    u, v = to_frame(layout, camera, x, y)
    x0, x1 = max(0, u - 44), min(width - LEGEND_WIDTH, u + 44)
    y0, y1 = max(HEADER_HEIGHT, v - 44), min(height, v + 44)
    crop = frame[y0:y1, x0:x1].astype(int)
    distance = np.sqrt(np.sum((crop - np.array(colour, dtype=int)) ** 2, axis=2))
    return int((distance < 90).sum())


def alignment_report(full_path, layout, camera, run_log, frame_times):
    """Reports how far each projected robot label sits from its rendered disc.
    @param full_path the encoded full video
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param run_log the run log
    @param frame_times the simulated time of every raw frame
    """
    capture = cv2.VideoCapture(full_path)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    largest = 0.0
    checked = 0
    for fraction in (0.1, 0.5, 0.9):
        index = int(total * fraction)
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = capture.read()
        if not ok or index >= len(frame_times):
            continue
        state = state_at(run_log, frame_times[index])
        if state is None:
            continue
        for robot in state["robots"]:
            colour = RENDERED_ROBOT_COLOURS_BGR[robot["id"] % len(RENDERED_ROBOT_COLOURS_BGR)]
            centre = detect_disc(frame, layout, camera, robot["x"], robot["y"], colour)
            if centre is None:
                continue
            u, v = to_frame(layout, camera, robot["x"], robot["y"])
            offset = float(np.hypot(centre[0] - u, centre[1] - v))
            largest = max(largest, offset)
            checked += 1
    capture.release()
    return largest, checked


def main():
    """Parses the arguments, renders the videos and prints the checks."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--full", required=True)
    parser.add_argument("--clip", required=True)
    parser.add_argument("--gif", required=True)
    parser.add_argument("--camera", required=True)
    parser.add_argument("--run-log", required=True)
    parser.add_argument("--frames-log", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--stills-dir", required=True)
    parser.add_argument("--stills-prefix", default="isaac_demo_still")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--clip-speed", type=int, default=8)
    parser.add_argument("--clip-scale", type=float, default=0.75)
    parser.add_argument("--gif-seconds", type=float, default=16.0)
    parser.add_argument("--gif-fps", type=float, default=5.0)
    parser.add_argument("--gif-width", type=int, default=480)
    parser.add_argument("--gif-colours", type=int, default=64)
    parser.add_argument("--gif-max-mb", type=float, default=8.0)
    args = parser.parse_args()

    camera = load_camera(args.camera)
    run_log = load_run_log(args.run_log)
    frame_times = load_frame_times(args.frames_log)
    map_data = load_map(args.map)
    layout = scene_layout(camera, map_data["width"], map_data["height"])
    total_orders = max((entry["released"] for entry in run_log), default=12)
    failures = 0

    print("run log: %d frames, t_abs %.2f to %.2f, total orders %d"
          % (len(run_log), run_log[0]["t_abs"], run_log[-1]["t_abs"], total_orders), flush=True)
    print("scene layout: crop %.0fx%.0f at (%.0f, %.0f), scale %.3f"
          % (layout["crop_w"], layout["crop_h"], layout["crop_x0"], layout["crop_y0"],
             layout["scale"]), flush=True)

    full_frames, full_width, full_height = render_video(
        args.input, args.full, args.fps, camera, layout, run_log, map_data, frame_times,
        total_orders, 1, 1.0, "1x")
    print(f"full: {full_width}x{full_height} {full_frames} frames "
          f"{full_frames / args.fps:.1f} s", flush=True)

    clip_frames, clip_width, clip_height = render_video(
        args.input, args.clip, args.fps, camera, layout, run_log, map_data, frame_times,
        total_orders, args.clip_speed, args.clip_scale, "%dx" % args.clip_speed)
    print(f"clip: {clip_width}x{clip_height} {clip_frames} frames "
          f"{clip_frames / args.fps:.1f} s at {args.clip_speed}x", flush=True)

    waits = collect_waits(run_log)
    summary = wait_summary(waits)
    for kind in ("cell_occupied", "pass_order"):
        print("waits %s: count %d, total %.1f s, longest %.1f s"
              % (kind, summary[kind]["count"], summary[kind]["total"],
                 summary[kind]["longest"]), flush=True)

    window = find_wait_window(run_log, waits, args.gif_seconds)
    if window is None:
        print("gif FAIL: no waiting robot found in the run log", flush=True)
        gif_frames = 0
        failures += 1
    else:
        gif_frames = render_gif(args.input, args.gif, camera, layout, run_log, map_data,
                                frame_times, total_orders, window, args.gif_fps, args.gif_width,
                                args.gif_colours, int(args.gif_max_mb * 1024 * 1024))
        wait = window[3]
        print("gif: starts at %.1f s, covers %.1f to %.1f s, holds %s for R%d from "
              "%.1f to %.1f s, %d frames"
              % (window[0], window[0], window[1], window[2], wait["robot"],
                 wait["start"], wait["end"], gif_frames), flush=True)

    for label, path in (("full", args.full), ("clip", args.clip)):
        info = probe(path)
        if info is None or info["frames"] == 0 or info["bad"] != 0:
            print(f"{label} decode FAIL", info, flush=True)
            failures += 1
            continue
        spreads = info["spread"]
        flat = sum(1 for entry in spreads if entry[1] < 5.0)
        print(f"{label} decode ok: {info['width']}x{info['height']} "
              f"{info['frames']} frames fps {info['fps']:.1f} "
              f"duration {info['frames'] / info['fps']:.1f} s bad {info['bad']} "
              f"chroma {chroma_format(path)} bytes {os.path.getsize(path)} "
              f"flat_frames {flat}/{len(spreads)}", flush=True)
        if flat == len(spreads):
            print(f"{label} has no visible content", flush=True)
            failures += 1

    if gif_frames:
        info = gif_probe(args.gif)
        print(f"gif decode ok: {info['width']}x{info['height']} {info['frames']} frames "
              f"duration {info['duration']:.1f} s bytes {info['bytes']}", flush=True)
        if info["frames"] == 0 or info["duration"] < 15.0 or info["bytes"] > int(
                args.gif_max_mb * 1024 * 1024):
            print("gif FAIL: bad frame count, too short or too large", flush=True)
            failures += 1
        wait = window[3]
        middle = (wait["start"] + wait["end"]) / 2.0
        state = state_at(run_log, middle)
        robot = next((entry for entry in state["robots"] if entry["id"] == wait["robot"]), None)
        index = min(range(len(frame_times)), key=lambda i: abs(frame_times[i] - middle))
        capture = cv2.VideoCapture(args.full)
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, marker_frame = capture.read()
        capture.release()
        if ok and robot is not None:
            occupied = marker_pixels(marker_frame, layout, camera, robot["x"], robot["y"],
                                     WAIT_OCCUPIED_COLOUR_BGR)
            right_of_way = marker_pixels(marker_frame, layout, camera, robot["x"], robot["y"],
                                         WAIT_RIGHT_OF_WAY_COLOUR_BGR)
            print("marker pixels at %.1f s (frame %d): next cell occupied %d, right of way %d"
                  % (frame_times[index], index, occupied, right_of_way), flush=True)

    previous = None
    edge_failures = 0
    for index, fraction in enumerate((0.1, 0.5, 0.9)):
        path = f"{args.stills_dir}/{args.stills_prefix}_{index}.png"
        info = still(args.full, fraction, path)
        if info is None:
            print(f"still {fraction} FAIL", flush=True)
            failures += 1
            continue
        print(f"still {fraction}: {info['path']} mean {info['mean']} std {info['std']} "
              f"min {info['min']} max {info['max']} overlay {info['overlay']}", flush=True)
        if info["std"] < 5.0 or info["overlay"] < 50:
            print(f"still {fraction} is flat or the overlay is missing", flush=True)
            failures += 1
        if previous is not None:
            difference = float(np.mean(cv2.absdiff(previous, info["gray"])))
            print(f"still {index} differs from previous by {difference:.2f}", flush=True)
            if difference < 0.2:
                failures += 1
        previous = info["gray"]
        edge_failures += count_edge_labels(path, layout, camera, map_data)
    if edge_failures:
        print(f"label edge check FAIL: {edge_failures} labels run past a frame edge", flush=True)
        failures += 1
    else:
        print("label edge check: no label runs past a frame edge in the three stills",
              flush=True)

    largest, checked = alignment_report(args.full, layout, camera, run_log, frame_times)
    print(f"alignment: {checked} robot labels checked, largest offset {largest:.1f} px",
          flush=True)
    if checked == 0 or largest > 12.0:
        print("alignment FAIL", flush=True)
        failures += 1

    if failures:
        print(f"video checks failed: {failures}", flush=True)
        return 1
    print("video checks passed", flush=True)
    return 0


def count_edge_labels(path, layout, camera, map_data):
    """Counts station labels in a still that would run past the scene area edge.
    @param path the still PNG
    @param layout the scene layout
    @param camera the camera framing dictionary
    @param map_data the map dictionary
    """
    frame = cv2.imread(path)
    if frame is None:
        return 1
    height, width = frame.shape[:2]
    scene_right = width - LEGEND_WIDTH
    bad = 0
    for name, x, y in map_data["stations"]:
        u, v = to_frame(layout, camera, x + 0.5, y + 0.5)
        origin = (min(max(u - 30, 2), scene_right - 70),
                  min(max(v + 34, HEADER_HEIGHT + 14), height - 6))
        (text_w, text_h), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        if origin[0] + text_w > scene_right or origin[1] > height or \
                origin[1] - text_h < HEADER_HEIGHT:
            bad += 1
    return bad


if __name__ == "__main__":
    sys.exit(main())
