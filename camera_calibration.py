#!/usr/bin/env python3
"""Intrinsic-калибровка RGB-камеры Intel RealSense D435i по ChArUco-доске.

Вход - dataset/NNNNNN_color.png (848x480 BGR8 от dataset_pipeline.py); IR и depth не участвуют,
геометрия кадров не меняется. Доска 9x6 клеток по 45 мм - те же 54 клетки, что названы 6x9; это
сетка 8x5 = 40 charuco-углов, а не 6x9 углов. Словарь DICT_4X4_50 определён перебором по кадрам:
ровно 27 маркеров, ID 0..26 = ceil(9*6/2). Маркер 33 мм - физический размер, а не подбор.

Object points берутся по charuco ID из board.chessboardCorners, а не по порядку детекции. Дисторсия
k1,k2,p1,p2 (CALIB_FIX_K3): slam.cpp читает ровно эти четыре, а свободный k3 перетянул бы на себя
k1,k2. Ключ --visualize строит визуальную диагностику уже посчитанной калибровки.
"""
import argparse, glob, os, sys, time
import cv2
import numpy as np

A = cv2.aruco
SQUARES_X, SQUARES_Y, SQUARE_M, MARKER_M = 9, 6, 0.045, 0.033  # физические размеры, не подбор
MIN_MARKERS, MIN_CORNERS, MAX_PLANAR_PX, PROBE_N, VAL_EVERY = 4, 8, 3.0, 6, 4

def load(root):
    """Чтение без единого изменения геометрии: калибровка идёт на исходных пикселях."""
    frames, size = [], None
    for p in sorted(glob.glob(os.path.join(root, "**", "*_color.png"), recursive=True)):
        im, why = cv2.imread(p, cv2.IMREAD_COLOR), None
        if im is None: why = "не читается"
        elif im.ndim != 3 or im.shape[2] != 3: why = f"не 3-канальный {im.shape}"
        elif size and (im.shape[1], im.shape[0]) != size: why = f"{im.shape[1]}x{im.shape[0]}"
        if why: print(f"  пропуск {os.path.basename(p)}: {why}"); continue
        size = (im.shape[1], im.shape[0]); frames.append((p, im, cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)))
    return frames, size

def detect(gray, board, params, flip, nmark):
    """ArUco -> refinement -> интерполяция charuco-углов. Своего cornerSubPix нет: его уже делает
    interpolateCornersCharuco, а лишний проход точность только ухудшал (0.6421 против 0.6176 при win=3)."""
    corners, ids, rej = A.detectMarkers(gray, board.dictionary, parameters=params)
    if ids is None or len(ids) < MIN_MARKERS: return None
    if flip: ids = (nmark - 1) - ids
    corners, ids, rej, _ = A.refineDetectedMarkers(gray, board, corners, ids, rej)
    n, cc, ci = A.interpolateCornersCharuco(corners, ids, gray, board)
    return (cc, ci) if n and n >= MIN_CORNERS else None

def planar_res(board, cc, ci):
    """Плоская доска: верные соответствия обязаны лечь на гомографию; ловит сбой раскладки. Среднее,
    а не медиана: при сбое раскладки уезжает меньшинство углов (5 из 40), медиана их не заметит."""
    obj = np.array(board.chessboardCorners)[ci.ravel()][:, :2].astype(np.float64)
    im = cc.reshape(-1, 2).astype(np.float64)
    H, _ = cv2.findHomography(obj, im, cv2.RANSAC, 3.0)
    if H is None: return np.inf
    proj = cv2.perspectiveTransform(obj.reshape(-1, 1, 2), H).reshape(-1, 2)
    return float(np.linalg.norm(proj - im, axis=1).mean())

def calibrate(CC, CI, board, size):  # pinhole + radial/tangential, k3 зафиксирован нулём
    return A.calibrateCameraCharucoExtended(CC, CI, board, size, None, None, flags=cv2.CALIB_FIX_K3)

def reproject(board, CC, CI, K, dist, rvecs, tvecs):
    # object points строго по charuco ID, поза кадра - из калибровки; отсюда же берёт диагностика
    obj = np.array(board.chessboardCorners, np.float32)
    det = [cc.reshape(-1, 2) for cc in CC]
    prj = [cv2.projectPoints(obj[ci.ravel()], r, t, K, dist)[0].reshape(-1, 2) for ci, r, t in zip(CI, rvecs, tvecs)]
    er = [np.linalg.norm(p - d, axis=1) for p, d in zip(prj, det)]
    return det, prj, er, np.array([float(np.sqrt((x ** 2).mean())) for x in er])

def holdout(board, CC, CI, K, dist, idx):
    """Кадры вне оценки K: внутренние параметры фиксированы, подбирается только поза (IPPE, плоская мишень)."""
    obj, errs = np.array(board.chessboardCorners, np.float32), []
    for j in idx:
        o, im = obj[CI[j].ravel()], CC[j].reshape(-1, 2)
        ok, rv, tv = cv2.solvePnP(o, im, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if ok: errs.append(np.linalg.norm(
            cv2.projectPoints(o, rv, tv, K, dist)[0].reshape(-1, 2) - im, axis=1))
    return np.concatenate(errs) if errs else np.array([np.nan])

def save(outdir, K, dist, meta):
    # OpenCV FileStorage: тот же формат, что читает cv::FileStorage в slam.cpp
    fs = cv2.FileStorage(os.path.join(outdir, "camera_calibration.yaml"), cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", K); fs.write("distortion_coefficients", dist)
    for k, v in meta.items(): fs.write(k, v)
    fs.release()
    with open(os.path.join(outdir, "slam_calib_rgb.yaml"), "w") as fh:  # slam_calib не трогаем
        fh.write("# RGB intrinsics из camera_calibration.py, вставить в slam_calib.yaml\n" + "".join(
            f"RGB.{k}: {v:.8f}\n" for k, v in zip(("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"),
                                                  (K[0,0], K[1,1], K[0,2], K[1,2], *dist.ravel()[:4]))))

FONT, VEC = cv2.FONT_HERSHEY_SIMPLEX, 20   # VEC - экранное усиление векторов, подписано на карте
_ecol = lambda e: (90,230,90) if e < .25 else (0,210,235) if e < .5 else (0,140,255) if e < 1 else (60,60,255)

def _txt(img, lines, y=16):
    for i, s in enumerate(lines): cv2.putText(img, s, (8, y+i*15), FONT, .42, (235,)*3, 1, cv2.LINE_AA)
    return img

def _canvas(size, pp, rmax):  # фон карт: граница кадра, оси центра, кольца по rmax, principal point
    W, H = size; img = np.full((H, W, 3), 24, np.uint8)
    cv2.line(img, (W//2, 0), (W//2, H), (55,)*3, 1); cv2.line(img, (0, H//2), (W, H//2), (55,)*3, 1)
    for f in (.25, .5, .75, 1.): cv2.circle(img, pp, int(f*rmax), (70,)*3, 1)
    cv2.rectangle(img, (0, 0), (W-1, H-1), (95,)*3, 1)
    cv2.drawMarker(img, pp, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
    return img

def diagnostics(root, frames, keep, det, prj, er, pf, K, dist, pve, size, meta, hold):
    """Диагностика уже посчитанной калибровки по её же принятым наблюдениям; ничего не меняет."""
    out = os.path.join(root, "diagnostics"); os.makedirs(out, exist_ok=True)
    D, P, E = np.concatenate(det), np.concatenate(prj), np.concatenate(er)
    cen = np.array([d.mean(0) for d in det]); W, H = size; d5 = dist.ravel(); dxy = (P - D).mean(0)
    cx, cy = float(K[0, 2]), float(K[1, 2]); pp = (int(cx), int(cy))
    rmax = max(np.hypot(x - cx, y - cy) for x in (0, W-1) for y in (0, H-1))   # PP -> угол кадра
    r = np.linalg.norm(D - [cx, cy], axis=1)
    bn = [int(((r >= a*rmax) & (r < b*rmax)).sum()) for a, b in ((0,.25), (.25,.5), (.5,.75), (.75,1.01))]
    x0, x1, y0, y1 = D[:, 0].min(), D[:, 0].max(), D[:, 1].min(), D[:, 1].max()
    m = [meta[k] for k in ("rms_reprojection_error", "mean_reprojection_error", "median_reprojection_error",
                           "max_reprojection_error", "mean_error_center", "mean_error_periphery")]
    cov = [f"observations {len(D)}   x {x0:.0f}..{x1:.0f}   y {y0:.0f}..{y1:.0f}   image {W}x{H}   "
           f"margins to edges: L {x0:.0f}  R {W-1-x1:.0f}  T {y0:.0f}  B {H-1-y1:.0f} px",
           f"rmax = principal point to image corner = {rmax:.1f} px;   max observed r = {r.max():.1f} px = {r.max()/rmax:.2f} rmax",
           f"radial bins of rmax:  0-25% {bn[0]}   25-50% {bn[1]}   50-75% {bn[2]}   75-100% {bn[3]}"]
    err = f"RMS {m[0]:.4f}  mean {m[1]:.4f}  median {m[2]:.4f}  max {m[3]:.4f}  center {m[4]:.4f}  periphery {m[5]:.4f} px"
    info = [f"resolution {W}x{H}   frames {meta['calibration_frames']}   observations {len(D)}   dict "
            f"{meta['aruco_dictionary']}   board {SQUARES_X}x{SQUARES_Y}   square {SQUARE_M}   marker {MARKER_M}",
            f"fx {K[0,0]:.5f}   fy {K[1,1]:.5f}   cx {cx:.5f}   cy {cy:.5f}",
            f"k1 {d5[0]:+.6f}   k2 {d5[1]:+.6f}   p1 {d5[2]:+.6f}   p2 {d5[3]:+.6f}   k3 {d5[4]:+.6f} fixed",
            err, hold] + cov
    T = {}
    o = np.argsort(pf)   # детерминированно: лучший, медианный, худший кадр и крайние положения доски
    for i in list(dict.fromkeys(int(k) for k in (o[0], o[len(o)//2], o[-1], *np.argmin(cen, 0), *np.argmax(cen, 0))))[:6]:
        nm = os.path.basename(frames[keep[i]][0])[:-4]; vis = frames[keep[i]][1].copy()
        cv2.polylines(vis, [cv2.convexHull(det[i].astype(np.float32)).astype(np.int32)], True, (200,160,0), 1)
        for (dx, dy), (px, py) in zip(det[i], prj[i]):  # кружок detected, крест projected, линия - невязка
            cv2.line(vis, (int(dx), int(dy)), (int(px), int(py)), (255,)*3, 1, cv2.LINE_AA)
            cv2.circle(vis, (int(round(dx)), int(round(dy))), 5, (90, 230, 90), 1, cv2.LINE_AA)
            cv2.drawMarker(vis, (int(round(px)), int(round(py))), (60, 60, 255), cv2.MARKER_CROSS, 7, 1)
        T["reprojection_" + nm] = _txt(vis, [f"{nm}   corners {len(er[i])}   frame RMS {pf[i]:.4f}   mean {er[i].mean():.4f}   max {er[i].max():.4f} px",
            "circle = detected, cross = projected, line = residual, hull = board outline"])
    v = _canvas(size, pp, rmax)
    for (dx, dy), (px, py) in zip(D, P):
        cv2.arrowedLine(v, (int(dx), int(dy)), (int(dx + (px-dx)*VEC), int(dy + (py-dy)*VEC)),
                        (0, 200, 255), 1, cv2.LINE_AA, tipLength=.35)
    T["reprojection_vectors"] = _txt(v, [f"detected -> projected displacement, vector display scale {VEC}x (real errors are sub-pixel)",
        f"systematic mean dx {dxy[0]:+.4f}   mean dy {dxy[1]:+.4f} px", err,
        f"cyan cross = principal point ({cx:.1f}, {cy:.1f}), rectangle = image boundary, rings = rmax/4"])
    for nm, rad, ec, lines in (("error_map", 3, 1, [f"reprojection error per accepted observation, n = {len(E)},"
            " measured points only, no interpolation", err,
            "green < 0.25    cyan < 0.5    orange < 1.0    red >= 1.0 px"]),
            ("coverage", 2, 0, ["spatial distribution of accepted observations"] + cov)):
        img = _canvas(size, pp, rmax)
        for (x, y), q in zip(D, E): cv2.circle(img, (int(x), int(y)), rad, _ecol(q) if ec else (90,230,90), -1)
        T[nm] = _txt(img, lines)
    j = int(np.argmax([len(x) for x in er])); src = frames[keep[j]][1]
    T["undistort_pair"] = _txt(np.hstack((src, cv2.undistort(src, K, dist))), [
        f"{os.path.basename(frames[keep[j]][0])}   left: original   right: undistorted with final K, dist",
        "straight edges are a visual check only, not ground truth"])
    rep = [k for k in T if k.startswith("reprojection_") and k != "reprojection_vectors"]
    cells = [cv2.resize(T[k], (354, 200)) for k in (rep[0], rep[-1], "coverage", "error_map", "reprojection_vectors", "undistort_pair")]
    sheet = np.vstack([np.hstack(cells[0:3]), np.hstack(cells[3:6])])
    T["calibration_visual_report"] = np.vstack((sheet, _txt(np.full((15*len(info)+22, sheet.shape[1], 3), 18, np.uint8), info)))
    for k, img in T.items(): cv2.imwrite(os.path.join(out, f"{k}.png"), img)
    bias = float(np.hypot(*dxy)); ratio = m[5] / max(m[4], 1e-9)
    verdict = ("FAIL" if bias > .25 or ratio > 4 or E.max() > 5 else
               "PASS" if bias < .05 and ratio < 2 and pf.max() < 1 else "PASS WITH OBSERVATIONS")
    return ["\nVISUAL DIAGNOSTICS", f"  каталог {out}: файлов {len(T)}, overlay-кадров {len(rep)}",
            f"  отчёт {os.path.join(out, 'calibration_visual_report.png')}"] + ["  " + s for s in cov] + [
            f"  systematic |mean (dx, dy)| {bias:.4f} px, periphery/center {ratio:.2f}, worst frame RMS {pf.max():.4f} px",
            f"  projectPoints против perViewErrors OpenCV: max расхождение {np.abs(pve - pf).max():.2e} px",
            f"  VISUAL VALIDATION: {verdict}"]

def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", default=os.path.join(here, "dataset"))
    ap.add_argument("--output", default=os.path.join(here, "calibration_output"))
    ap.add_argument("--dictionary", default="DICT_4X4_50"); ap.add_argument("--visualize", action="store_true")
    ap.add_argument("--force", action="store_true", help="перезаписать прежний результат")
    args = ap.parse_args()
    out_yaml = os.path.join(args.output, "camera_calibration.yaml")
    if os.path.exists(out_yaml) and not args.force: sys.exit(f"{out_yaml} уже есть: --force или --output")
    if not hasattr(A, args.dictionary): sys.exit(f"нет словаря {args.dictionary}")
    board = A.CharucoBoard_create(SQUARES_X, SQUARES_Y, SQUARE_M, MARKER_M, A.Dictionary_get(getattr(A, args.dictionary)))
    nmark, params = len(np.array(board.ids).ravel()), A.DetectorParameters_create()
    params.cornerRefinementMethod = A.CORNER_REFINE_SUBPIX
    frames, size = load(args.dataset)
    if not frames: sys.exit(f"нет читаемых *_color.png в {args.dataset}")  # RGB, не IR/depth
    print(f"кадров прочитано {len(frames)}, разрешение {size[0]}x{size[1]}, доска {SQUARES_X}x{SQUARES_Y} по "
          f"{SQUARE_M*1000:.0f} мм -> {(SQUARES_X-1)*(SQUARES_Y-1)} charuco-углов, словарь {args.dictionary}")
    # Доска напечатана в новой раскладке OpenCV (>=4.6), а 4.5.4 строит legacy-доску с обратной
    # чётностью клеток под маркеры; повёрнутая на 180° новая доска и есть legacy, поворот меняет
    # id -> N-1-id и на intrinsics не влияет. Иначе соответствия битые: было 14 из 35, RMS 41 px.
    med = {}
    for f in (False, True):
        r = [planar_res(board, *d) for _, _, g in frames[::max(1, len(frames)//PROBE_N)] if (d := detect(g, board, params, f, nmark))]
        med[f] = float(np.median(r)) if r else np.inf
    flip = med[True] < med[False]
    print(f"раскладка доски: {'новая OpenCV>=4.6, id -> N-1-id' if flip else 'legacy'} "
          f"(остаток гомографии legacy {med[False]:.2f} px, новая {med[True]:.2f} px)")
    CC, CI, keep = [], [], []
    for j, (p, _, g) in enumerate(frames):
        d = detect(g, board, params, flip, nmark); r = planar_res(board, *d) if d else np.inf
        why = f"доска не найдена / < {MIN_CORNERS} углов" if d is None else f"не плоско, {r:.1f} px"
        if r > MAX_PLANAR_PX: print(f"  отброшен {os.path.basename(p)}: {why}")
        else: CC.append(d[0]); CI.append(d[1]); keep.append(j)
    print(f"доска найдена на {len(CC)} кадрах, углов {sum(len(c) for c in CC)}")
    if len(CC) < 6: sys.exit("слишком мало валидных кадров")
    rms, K, dist, rv, tv, _, _, pve = calibrate(CC, CI, board, size)
    pf = pve.ravel()  # робастный порог: медиана + 3 сигмы по MAD, без произвольных констант
    thr = float(np.median(pf) + 3 * 1.4826 * np.median(np.abs(pf - np.median(pf))))  # 1.4826: MAD -> сигма
    good = [i for i in range(len(CC)) if pf[i] <= thr]
    print(f"стартовая калибровка: RMS {rms:.4f} px по {len(CC)} кадрам, порог {thr:.3f} px")
    for i in sorted(set(range(len(CC))) - set(good)):
        print(f"  выброс {os.path.basename(frames[keep[i]][0])}: ошибка кадра {pf[i]:.2f} px")
    if 6 <= len(good) < len(CC):
        CC, CI, keep = [CC[i] for i in good], [CI[i] for i in good], [keep[i] for i in good]
        rms, K, dist, rv, tv, _, _, pve = calibrate(CC, CI, board, size)
    det, prj, er, pf = reproject(board, CC, CI, K, dist, rv, tv)
    errs, pts = np.concatenate(er), np.concatenate(det)
    val = list(range(VAL_EVERY-1, len(CC), VAL_EVERY)); train = [i for i in range(len(CC)) if i not in val]  # без rng
    if len(val) >= 2 and len(train) >= 6:
        _, Kt, dt, *_ = calibrate([CC[i] for i in train], [CI[i] for i in train], board, size)
        hv, ht = holdout(board, CC, CI, Kt, dt, val), holdout(board, CC, CI, Kt, dt, train)
        hold = (f"intra-dataset hold-out: train {len(train)} -> val {len(val)} frames, val mean {hv.mean():.4f} px, "
                f"train mean {ht.mean():.4f} px, fx shift {abs(Kt[0,0]-K[0,0]):.2f} px, cx shift {abs(Kt[0,2]-K[0,2]):.2f} px")
    else: hold = "intra-dataset hold-out: too few frames, validation on the full set only"
    edge = np.linalg.norm(pts - [K[0, 2], K[1, 2]], axis=1) > 0.35 * np.hypot(*size)
    meta = {f"{n}_reprojection_error": float(v) for n, v in (
        ("rms", rms), ("mean", errs.mean()), ("median", np.median(errs)), ("max", errs.max()))}
    meta.update(mean_error_center=float(errs[~edge].mean()), mean_error_periphery=float(errs[edge].mean()),
                median_frame_error=float(np.median(pf)), max_frame_error=float(pf.max()),
                image_width=size[0], image_height=size[1], calibration_frames=len(CC), square_size_m=SQUARE_M,
                charuco_corners=int(len(errs)), distortion_model="plumb_bob k1 k2 p1 p2 (k3 fixed to zero)",
                board_squares_x=SQUARES_X, board_squares_y=SQUARES_Y, aruco_dictionary=args.dictionary,
                marker_size_m=MARKER_M, opencv_version=cv2.__version__, holdout_validation=hold,
                board_layout="opencv>=4.6" if flip else "legacy", calibration_time=time.strftime("%Y-%m-%dT%H:%M:%S"))
    bad = [n for n, ok in (
        ("fx/fy не положительны", K[0, 0] > 0 and K[1, 1] > 0),
        ("principal point вне центра", 0.3 * size[0] < K[0, 2] < 0.7 * size[0] and 0.3 * size[1] < K[1, 2] < 0.7 * size[1]),
        ("NaN/Inf", np.isfinite(K).all() and np.isfinite(dist).all() and np.isfinite(errs).all()),
        ("форма K/dist", K.shape == (3, 3) and dist.size == 5),
        ("число соответствий не сходится", sum(len(c) for c in CC) == len(errs)),
        ("RMS расходится с пересчётом projectPoints", abs(rms - np.sqrt((errs ** 2).mean())) < 1e-3),
        ("charuco ID вне сетки доски", all(0 <= i < (SQUARES_X-1)*(SQUARES_Y-1) for ci in CI for i in ci.ravel()))) if not ok]
    if bad: sys.exit("sanity check не пройден: " + ", ".join(bad))
    print(f"\nK:\n{np.array2string(K, precision=3, suppress_small=True)}\ndist k1 k2 p1 p2 k3: "
          f"{np.round(dist.ravel(), 6).tolist()}\nпринято кадров {len(CC)}, углов {len(errs)}\nошибка "
          "перепроекции, px: " + ", ".join(f"{k} {meta[k]:.4f}" for k in meta if "error" in k) + f"\n{hold}")
    os.makedirs(args.output, exist_ok=True)
    save(args.output, K, dist, meta)
    print(f"\nрезультат: {out_yaml}")
    if args.visualize:
        for s in diagnostics(args.output, frames, keep, det, prj, er, pf, K, dist, pve.ravel(),
                             size, meta, hold): print(s)
    return 0
if __name__ == "__main__":
    sys.exit(main())
