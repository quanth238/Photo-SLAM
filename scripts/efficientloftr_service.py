#!/usr/bin/env python3

import argparse
import json
import os
import sys
import cv2
import numpy as np
import torch
import zmq

from copy import deepcopy

# Ensure EfficientLoFTR repo is on PYTHONPATH
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ELOFTR_DIR = os.path.join(ROOT, "reference", "EfficientLoFTR")
if ELOFTR_DIR not in sys.path:
    sys.path.append(ELOFTR_DIR)
    sys.path.append(os.path.join(ELOFTR_DIR, "src"))

from src.loftr import LoFTR, full_default_cfg, reparameter


def resize_to_multiple_of_32(img):
    h, w = img.shape[:2]
    h1 = max(32, (h // 32) * 32)
    w1 = max(32, (w // 32) * 32)
    if h1 == h and w1 == w:
        return img, 1.0, 1.0
    img_resized = cv2.resize(img, (w1, h1), interpolation=cv2.INTER_LINEAR)
    sx = w / float(w1)
    sy = h / float(h1)
    return img_resized, sx, sy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", type=str, default="tcp://127.0.0.1:5555")
    parser.add_argument("--weights", type=str, default="weights/eloftr_outdoor.ckpt")
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    cfg = deepcopy(full_default_cfg)
    matcher = LoFTR(config=cfg)
    try:
        ckpt = torch.load(args.weights, map_location=args.device, weights_only=False)
    except TypeError:
        # Older torch versions without weights_only param
        ckpt = torch.load(args.weights, map_location=args.device)
    matcher.load_state_dict(ckpt["state_dict"])
    matcher = reparameter(matcher)
    matcher = matcher.eval().to(args.device)

    ctx = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.bind(args.endpoint)

    while True:
        parts = socket.recv_multipart()
        if len(parts) < 3:
            continue
        header = json.loads(parts[0].decode("utf-8"))
        h0 = int(header["h0"])
        w0 = int(header["w0"])
        h1 = int(header["h1"])
        w1 = int(header["w1"])

        img0 = np.frombuffer(parts[1], dtype=np.uint8).reshape(h0, w0)
        img1 = np.frombuffer(parts[2], dtype=np.uint8).reshape(h1, w1)

        img0_resized, sx0, sy0 = resize_to_multiple_of_32(img0)
        img1_resized, sx1, sy1 = resize_to_multiple_of_32(img1)

        t0 = torch.from_numpy(img0_resized)[None][None].to(args.device).float() / 255.0
        t1 = torch.from_numpy(img1_resized)[None][None].to(args.device).float() / 255.0
        batch = {"image0": t0, "image1": t1}

        with torch.no_grad():
            matcher(batch)
            mkpts0 = batch["mkpts0_f"].cpu().numpy().astype(np.float32)
            mkpts1 = batch["mkpts1_f"].cpu().numpy().astype(np.float32)
            mconf = batch["mconf"].cpu().numpy().astype(np.float32)

        # Rescale back to original undistorted resolution (Option A)
        mkpts0[:, 0] *= sx0
        mkpts0[:, 1] *= sy0
        mkpts1[:, 0] *= sx1
        mkpts1[:, 1] *= sy1

        resp_header = json.dumps({"req_id": header.get("req_id", 0),
                                  "num_matches": int(mkpts0.shape[0])})

        socket.send_multipart([
            resp_header.encode("utf-8"),
            mkpts0.tobytes(),
            mkpts1.tobytes(),
            mconf.tobytes()
        ])


if __name__ == "__main__":
    main()
