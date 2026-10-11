"""Turns a raw simulator recording into a widely supported H.264 mp4 with the
index at the front, writes a smaller clip and three stills, and reports the full
decode result. Runs on the host because the simulator image cannot encode H.264.
"""

import argparse
import sys

import cv2
import numpy as np

CONTAINER_BOXES = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"dinf", b"edts"}


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


def encode(raw_path, out_path, fps):
    """Re-encodes a raw recording to H.264 and returns (frames, width, height).
    @param raw_path the raw video to read
    @param out_path the H.264 mp4 to write
    @param fps the frame rate of the result
    """
    source = cv2.VideoCapture(raw_path)
    if not source.isOpened():
        raise RuntimeError("cannot open " + raw_path)
    width = int(source.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(source.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"avc1"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("the host OpenCV cannot encode H.264")
    frames = 0
    while True:
        ok, frame = source.read()
        if not ok:
            break
        writer.write(frame)
        frames += 1
    source.release()
    writer.release()
    rewrite_faststart(out_path)
    return frames, width, height


def make_clip(full_path, clip_path, seconds, fps, scale):
    """Writes the first part of the full video as a smaller H.264 clip.
    @param full_path the encoded full video
    @param clip_path the clip to write
    @param seconds how many seconds to keep
    @param fps the clip frame rate
    @param scale the clip size as a fraction of the full size
    """
    source = cv2.VideoCapture(full_path)
    if not source.isOpened():
        raise RuntimeError("cannot open " + full_path)
    width = int(source.get(cv2.CAP_PROP_FRAME_WIDTH) * scale)
    height = int(source.get(cv2.CAP_PROP_FRAME_HEIGHT) * scale)
    writer = cv2.VideoWriter(clip_path, cv2.VideoWriter_fourcc(*"avc1"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("the host OpenCV cannot encode H.264")
    limit = seconds * fps
    frames = 0
    while frames < limit:
        ok, frame = source.read()
        if not ok:
            break
        writer.write(cv2.resize(frame, (width, height)))
        frames += 1
    source.release()
    writer.release()
    rewrite_faststart(clip_path)
    return frames, width, height


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


def still(path, fraction, out_path):
    """Saves one frame at a fraction through a video as a PNG and returns stats.
    @param path the video to read
    @param fraction where to take the frame, 0..1
    @param out_path the PNG to write
    """
    capture = cv2.VideoCapture(path)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(total * fraction))
    ok, frame = capture.read()
    capture.release()
    if not ok or frame is None:
        return None
    cv2.imwrite(out_path, frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {"mean": round(float(gray.mean()), 2), "std": round(float(gray.std()), 2),
            "min": int(gray.min()), "max": int(gray.max()), "path": out_path,
            "gray": gray}


def main():
    """Parses the arguments, encodes the videos and prints the checks."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--full", required=True)
    parser.add_argument("--clip", required=True)
    parser.add_argument("--stills-dir", required=True)
    parser.add_argument("--stills-prefix", default="isaac_demo_still")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--clip-seconds", type=float, default=60.0)
    parser.add_argument("--clip-scale", type=float, default=0.5)
    args = parser.parse_args()

    frames, width, height = encode(args.input, args.full, args.fps)
    print(f"full: {width}x{height} {frames} frames "
          f"{frames / args.fps:.1f} s", flush=True)
    clip_frames, clip_width, clip_height = make_clip(
        args.full, args.clip, args.clip_seconds, args.fps, args.clip_scale)
    print(f"clip: {clip_width}x{clip_height} {clip_frames} frames "
          f"{clip_frames / args.fps:.1f} s", flush=True)

    failures = 0
    for label, path in (("full", args.full), ("clip", args.clip)):
        info = probe(path)
        if info is None or info["frames"] == 0 or info["bad"] != 0:
            print(f"{label} decode FAIL", info, flush=True)
            failures += 1
            continue
        spreads = info["spread"]
        flat = sum(1 for s in spreads if s[1] < 5.0)
        print(f"{label} decode ok: {info['width']}x{info['height']} "
              f"{info['frames']} frames fps {info['fps']:.1f} "
              f"duration {info['frames'] / info['fps']:.1f} s bad {info['bad']} "
              f"flat_frames {flat}/{len(spreads)}", flush=True)
        if flat == len(spreads):
            print(f"{label} has no visible content", flush=True)
            failures += 1

    previous = None
    for index, fraction in enumerate((0.1, 0.5, 0.9)):
        path = f"{args.stills_dir}/{args.stills_prefix}_{index}.png"
        info = still(args.full, fraction, path)
        if info is None:
            print(f"still {fraction} FAIL", flush=True)
            failures += 1
            continue
        print(f"still {fraction}: {info['path']} mean {info['mean']} "
              f"std {info['std']} min {info['min']} max {info['max']}", flush=True)
        if info["std"] < 5.0:
            print(f"still {fraction} is a flat colour", flush=True)
            failures += 1
        if previous is not None:
            difference = float(np.mean(cv2.absdiff(previous, info["gray"])))
            print(f"still {index} differs from previous by {difference:.2f}", flush=True)
            if difference < 0.5:
                failures += 1
        previous = info["gray"]

    if failures:
        print(f"video checks failed: {failures}", flush=True)
        return 1
    print("video checks passed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
