#!/usr/bin/env python3
"""在 Jetson / 任意 aarch64 机器上直接跑深大装甲板模型（不依赖 ROS）

用途：ROS 版本还没搞定之前，先在小电脑上验证"模型效果 + 推理耗时"。
用的预处理/解码逻辑和 C++ 节点（neural_detector.cpp）完全一致：
    拉伸到 640x640 → RGB → 0~1 → NCHW(fp16/fp32 自动) → 25200x22 解码 → NMS

用法：
    python3 jetson_nn_demo.py <图片或视频> [模型路径] [输出路径] [--red]

例子：
    python3 jetson_nn_demo.py blue_rotate_fast.mp4
    python3 jetson_nn_demo.py blue_rotate_fast.mp4 shenzhen-0526.onnx /tmp/out.avi
    python3 jetson_nn_demo.py red_rotate_fast.mp4 shenzhen-0526.onnx /tmp/out.avi --red

依赖（Jetson 上这样装）：
    pip3 install onnxruntime          # aarch64 轮子，CPU 推理
    sudo apt install python3-opencv   # cv2 用 apt 的，别用 pip 装
"""

import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

INPUT_W, INPUT_H = 640, 640
CONF_COL, COLOR_COL, GENRE_COL = 8, 9, 13

# 类别序号 → (编号, 是否大装甲板)，和 neural_detector.cpp 的 kGenres 一致
GENRES = [
    ("guard", False),    # 0 哨兵
    ("1", True),         # 1 英雄（大装甲板）
    ("2", False),        # 2 工程
    ("3", False),
    ("4", False),
    ("5", False),
    ("outpost", False),  # 6 前哨站
    ("base", False),     # 7 基地小
    ("base", True),      # 8 基地大
]
COLOR_NAMES = ["blue", "red", "dark", "mix"]


def preprocess(rgb, dtype):
    """拉伸到 640x640 → RGB/255 → NCHW，返回 (1,3,H,W) 的 dtype 数组"""
    resized = cv2.resize(rgb, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
    x = resized.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))[None]  # HWC -> 1CHW
    return np.ascontiguousarray(x.astype(dtype))


def detect(session, rgb, conf_thres=0.65, nms_thres=0.45, detect_color=0):
    input_meta = session.get_inputs()[0]
    dtype = np.float16 if "float16" in input_meta.type else np.float32
    x = preprocess(rgb, dtype)

    t0 = time.time()
    out = session.run(None, {input_meta.name: x})[0]  # (1, 25200, 22)
    infer_ms = (time.time() - t0) * 1000.0

    rows = out[0]
    conf = 1.0 / (1.0 + np.exp(-rows[:, CONF_COL].astype(np.float32)))
    keep = conf >= conf_thres
    if not keep.any():
        return [], infer_ms

    scale_x = rgb.shape[1] / INPUT_W
    scale_y = rgb.shape[0] / INPUT_H

    boxes, scores, dets = [], [], []
    for row, c in zip(rows[keep], conf[keep]):
        color_id = int(np.argmax(row[COLOR_COL:COLOR_COL + 4]))
        if color_id >= 2 or color_id != detect_color:  # 灰/紫丢掉；只留要打的那个颜色
            continue
        genre = int(np.argmax(row[GENRE_COL:GENRE_COL + 9]))
        pts = np.array(
            [[row[i * 2] * scale_x, row[i * 2 + 1] * scale_y] for i in range(4)], dtype=np.float32)
        x1, y1 = pts.min(axis=0)
        x2, y2 = pts.max(axis=0)
        boxes.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
        scores.append(float(c))
        dets.append((pts, genre, color_id, float(c)))

    if not boxes:
        return [], infer_ms

    idx = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, nms_thres)
    result = [dets[i] for i in np.array(idx).flatten()]
    return result, infer_ms


def draw(img, dets):
    for pts, genre, color_id, conf in dets:
        number, large = GENRES[genre]
        color = (255, 128, 0) if color_id == 0 else (0, 128, 255)  # BGR：蓝/红
        cv2.polylines(img, [pts.astype(np.int32)], True, color, 2)
        for p in pts:
            cv2.circle(img, (int(p[0]), int(p[1])), 3, (0, 255, 255), -1)
        cv2.putText(
            img, f"{number}{'(L)' if large else ''}: {conf * 100:.1f}%",
            (int(pts[0][0]), int(pts[0][1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return img


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    src = sys.argv[1]
    model = sys.argv[2] if len(sys.argv) > 2 else "shenzhen-0526.onnx"
    out_path = sys.argv[3] if len(sys.argv) > 3 else ""
    detect_color = 1 if "--red" in sys.argv else 0

    session = ort.InferenceSession(model, providers=["CPUExecutionProvider"])
    print(f"模型: {model}  输入: {session.get_inputs()[0].shape} {session.get_inputs()[0].type}")
    print(f"识别颜色: {'红' if detect_color else '蓝'}")

    image = cv2.imread(src)
    if image is not None:  # 图片
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        dets, ms = detect(session, rgb, detect_color=detect_color)
        print(f"图片 {src}: 检出 {len(dets)} 个装甲板, 推理 {ms:.1f} ms")
        for pts, genre, color_id, conf in dets:
            print(f"  {GENRES[genre][0]} ({COLOR_NAMES[color_id]}) conf={conf:.3f} "
                  f"角点={np.round(pts, 1).tolist()}")
        out = out_path or src + "_nn.jpg"
        cv2.imwrite(out, draw(image, dets))
        print(f"结果图: {out}")
        return 0

    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"打不开: {src}")
        return 1

    fps = cap.get(cv2.CAP_PROP_FPS)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = out_path or src + "_nn.avi"
    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"MJPG"), fps if fps > 1 else 25, (w, h))

    frames = hit_frames = total = 0
    total_ms = 0.0
    while True:
        ok, bgr = cap.read()
        if not ok or bgr is None:
            break
        dets, ms = detect(session, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), detect_color=detect_color)
        frames += 1
        total_ms += ms
        if dets:
            hit_frames += 1
            total += len(dets)
        writer.write(draw(bgr, dets))

    writer.release()
    print(f"视频 {src}: {frames} 帧, 有检出 {hit_frames} 帧, 装甲板 {total} 个, "
          f"平均推理 {total_ms / max(frames, 1):.1f} ms (约 {1000 * frames / max(total_ms, 1e-6):.1f} FPS 纯推理)")
    print(f"结果视频: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
