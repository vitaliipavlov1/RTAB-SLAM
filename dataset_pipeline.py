"""Ручная съёмка RAW-датасета с Intel RealSense D435i.
This script only captures raw data. It does not calibrate anything and does not detect
any target: ни ChArUco, ни ArUco, ни шахматной доски. Годность кадра решает только
человек. Поток: live -> SPACE фиксирует frameset -> Y/ENTER сохранить, N/ESC отбросить.
"""

import argparse, json, os, sys, time

import cv2
import numpy as np
import pyrealsense2 as rs

WIN, PANEL, GRID_W, ESC, SPACE, ENTER = "D435i - raw capture", 76, 1024, 27, 32, 13
TAGS = ("ir_left", "ir_right", "color", "depth")
GREEN, WHITE, YELLOW, FONT = (120, 255, 120), (215,) * 3, (0, 230, 255), cv2.FONT_HERSHEY_SIMPLEX

def store(out, mp, root, frames, index, im, meta) -> bool:
    # четыре PNG плюс запись в metadata: либо всё целиком, либо ничего
    def drop(paths):  # откат: отсутствие файла ошибкой не считаем
        for p in paths:
            try: os.remove(p)
            except OSError: pass
    st = root["streams"]
    want = {t: ((st[t]["height"], st[t]["width"]) + ((3,) if t == "color" else ()),
                np.uint16 if t == "depth" else np.uint8) for t in TAGS}
    for t in TAGS:
        if im[t].shape != want[t][0] or im[t].dtype != want[t][1]:
            print(f"  {t}: {im[t].shape} {im[t].dtype}, ожидалось {want[t][0]} "
                  f"{np.dtype(want[t][1]).name}; не записано, snapshot цел")
            return False
    written = []
    for tag in TAGS:
        path = os.path.join(out, f"{index:06d}_{tag}.png")
        if os.path.exists(path):  # чужой файл не трогаем и не откатываем
            drop(written); print(f"  {index:06d}_{tag}.png уже существует, набор не записан")
            return False
        try:
            ok = cv2.imwrite(path, im[tag])
        except Exception as exc:
            print(f"  imwrite упал: {exc}")
            ok = False
        if not ok:  # imwrite мог оставить частичный файл - убираем и его
            drop(written + [path])
            print(f"  ошибка записи {index:06d}: откат набора, metadata не тронута")
            return False
        written.append(path)
    frames.append(dict(meta, frame=index, session=root["session"],
                       files={t: os.path.basename(p) for t, p in zip(TAGS, written)}))
    root["frames"] = frames
    try:  # tmp + atomic replace: прежняя metadata переживёт сбой записи
        with open(mp + ".tmp", "w", encoding="utf-8") as fh:
            json.dump(root, fh, ensure_ascii=False, indent=1)
        os.replace(mp + ".tmp", mp)
    except Exception as exc:
        frames.pop(); drop(written + [mp + ".tmp"])
        print(f"  metadata не записана ({exc}), набор откачен")
        return False
    return True

def preview(im, info, scale):
    # уменьшение только для экрана; на диск идут исходные кадры без обработки
    tiles = []
    for tag in TAGS:
        if tag == "depth":
            v = cv2.applyColorMap(cv2.convertScaleAbs(im[tag], None, 0.05), cv2.COLORMAP_JET)
            v[im[tag] == 0] = 0
        else:
            v = im[tag] if tag == "color" else cv2.cvtColor(im[tag], cv2.COLOR_GRAY2BGR)
        small = cv2.resize(v, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.putText(small, tag.replace("_", " ").upper(), (8, 18), FONT, 0.45, WHITE, 1)
        tiles.append(small)
    panel = np.zeros((PANEL, tiles[0].shape[1] * 2, 3), np.uint8)
    for i, (text, color) in enumerate(info):
        cv2.putText(panel, text, (10, 20 + i * 22), FONT, 0.5, color, 1, cv2.LINE_AA)
    return np.vstack((np.hstack(tiles[:2]), np.hstack(tiles[2:]), panel))

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag, kind, default in (("--width", int, 848), ("--height", int, 480),
                                ("--fps", int, 30), ("--out", str, "dataset")):
        ap.add_argument(flag, type=kind, default=default)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    mp = os.path.join(args.out, "metadata.json")
    try:
        frames = json.load(open(mp, encoding="utf-8"))["frames"] if os.path.exists(mp) else []
    except (ValueError, KeyError, OSError):
        sys.exit(f"{mp} повреждён: почините или уберите, иначе потеряются прежние записи")
    nums = [int(n[:6]) for n in os.listdir(args.out) if n[:6].isdigit() and n.endswith(".png")]
    index = max(nums) + 1 if nums else 1  # пропуски и неполные наборы номер не переиспользуют
    pipe, sensor, was_emitter, proj, saved, times, shot = None, None, None, "unknown", 0, [], None
    try:
        cfg, wh, f = rs.config(), (args.width, args.height), args.fps
        cfg.enable_stream(rs.stream.color, *wh, rs.format.bgr8, f)
        cfg.enable_stream(rs.stream.depth, *wh, rs.format.z16, f)
        for idx in (1, 2): cfg.enable_stream(rs.stream.infrared, idx, *wh, rs.format.y8, f)
        p = rs.pipeline(); profile = p.start(cfg); pipe = p
        dev = profile.get_device()
        sensor = dev.first_depth_sensor()
        if sensor.supports(rs.option.emitter_enabled):
            was_emitter, proj = sensor.get_option(rs.option.emitter_enabled), "OFF"
            sensor.set_option(rs.option.emitter_enabled, 0.0)
        else: proj = "unsupported"
        vs = {t: profile.get_stream(st, i).as_video_stream_profile() for t, st, i in (
            ("ir_left", rs.stream.infrared, 1), ("ir_right", rs.stream.infrared, 2),
            ("color", rs.stream.color, 0), ("depth", rs.stream.depth, 0))}
        streams = {t: {"width": v.width(), "height": v.height(), "fps": v.fps(),
                       "format": str(v.format()).split(".")[-1]} for t, v in vs.items()}
        root = {k: dev.get_info(getattr(rs.camera_info, a)) for k, a in (
            ("camera", "name"), ("serial", "serial_number"), ("firmware", "firmware_version"))}
        root.update(session=time.strftime("%Y%m%dT%H%M%S"), streams=streams,
                    depth_scale=sensor.get_depth_scale())
        s = streams["ir_left"]
        h, w, scale = s["height"], s["width"], GRID_W / 2 / s["width"]
        head = f"{root['camera']} {root['serial']} fw {root['firmware']}  {w}x{h}@{s['fps']}"
        print(json.dumps(root, ensure_ascii=False, indent=1) + f"\n  сессия {root['session']}, "
              f"набор {index:06d}\n  SPACE снять | Y/ENTER сохранить | N/ESC отбросить\n")
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.imshow(WIN, np.zeros((int(h * scale) * 2 + PANEL, int(w * scale) * 2, 3), np.uint8))
        cv2.resizeWindow(WIN, int(w * scale) * 2, int(h * scale) * 2 + PANEL)
        while True:
            if shot is None:
                try:
                    fs = pipe.wait_for_frames(2000)
                except RuntimeError:
                    print("  камера не отдаёт кадры, выхожу"); break
                got = (fs.get_infrared_frame(1), fs.get_infrared_frame(2),
                       fs.get_color_frame(), fs.get_depth_frame())
                if not all(got): continue
                times = (times + [now := time.monotonic()])[-30:]
                im = {t: np.asanyarray(fr.get_data()) for t, fr in zip(TAGS, got)}
                meta = {"host_monotonic_s": now, "rs_timestamp_ms": fs.get_timestamp(),
                        "rs_frame_number": fs.get_frame_number(),
                        "timestamp_domain": str(fs.get_frame_timestamp_domain()).split(".")[-1]}
                fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
                valid = np.count_nonzero(im["depth"]) / im["depth"].size * 100
            src, m = shot if shot else (im, meta)
            cv2.imshow(WIN, preview(src, [
                ("LIVE   SPACE снять кадр, ESC выход", GREEN) if shot is None else
                ("CAPTURED FRAME - SAVE (Y/ENTER) OR DISCARD (N/ESC)", YELLOW),
                (f"frame {m['rs_frame_number']}   rs ts {m['rs_timestamp_ms']:.0f} ms "
                 f"({m['timestamp_domain']})   host {m['host_monotonic_s']:.3f} s   "
                 f"FPS {fps:.1f}   depth valid {valid:.1f}%", WHITE),
                (f"{head}  projector {proj}  saved {saved}  next {index:06d}", WHITE)], scale))
            key = cv2.waitKey(1 if shot is None else 30) & 0xFF
            if shot is None:
                if key == ESC: break
                if key == SPACE:  # копии: буфер кадра RealSense переиспользует
                    shot = ({t: im[t].copy() for t in TAGS}, meta)
            elif key in (ord("y"), ENTER) and store(args.out, mp, root, frames, index, *shot):
                print(f"  сохранён набор {index:06d}")
                index, saved, shot = index + 1, saved + 1, None
            elif key in (ord("n"), ESC):
                shot = None; print("  кадр отброшен, на диск ничего не записано")
    except KeyboardInterrupt:
        print("\n  прервано")
    finally:
        if was_emitter is not None:
            try: sensor.set_option(rs.option.emitter_enabled, was_emitter)
            except Exception as exc: print(f"  проектор не восстановлен: {exc}")
        if pipe is not None:
            try: pipe.stop()
            except Exception as exc: print(f"  pipeline не остановлен: {exc}")
        cv2.destroyAllWindows()
    print(f"\n  снято {saved}, в metadata.json {len(frames)}, датасет {os.path.abspath(args.out)}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
