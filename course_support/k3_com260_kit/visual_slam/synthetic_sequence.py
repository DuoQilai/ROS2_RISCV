"""Render a calibrated monocular sequence, matching IMU samples and ground truth."""
import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

SIZE = (640, 480)
K = np.array([[440., 0, 319.5], [0, 440., 239.5], [0, 0, 1.]])
BODY_FROM_CAMERA = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.]])


def position(t):
    return np.array([
        .80 * np.sin(.67*t) + .32 * np.sin(1.23*t),
        .72 * np.sin(.58*t) + .35 * np.sin(1.42*t),
        1.35 + .2 * np.cos(.83*t) + .075 * np.cos(1.76*t),
    ])


def orientation(t):
    return Rotation.from_euler("ZYX", [.2*np.sin(.55*t), .10*np.sin(.81*t), .085*np.cos(.63*t)])


def textured_plane(rng, x, y, z, width, height):
    # Four world-space corners, ordered like the corners of the texture image.
    dy, dz = width/2, height/2
    corners = np.array([[x,y+dy,z+dz], [x,y-dy,z+dz], [x,y-dy,z-dz], [x,y+dy,z-dz]])
    shape = (max(50, int(100*height)), max(50, int(100*width)))
    texture = np.full(shape, 100, np.float32)
    for cell, amplitude in [(64, 45), (16, 25), (4, 20), (1, 7)]:
        grid = (max(2, shape[0]//cell), max(2, shape[1]//cell))
        noise = rng.normal(0, 1, grid).astype(np.float32)
        texture += amplitude * cv2.resize(noise, shape[::-1], interpolation=cv2.INTER_CUBIC)
    texture = np.uint8(texture.clip(8, 247))
    for _ in range(int(width*height*2.5)):
        a = rng.integers([0, 0], shape[::-1])
        r = int(rng.integers(3, 15))
        color = int(rng.integers(20, 230))
        cv2.rectangle(texture, tuple(a), tuple(a+[r, 2*r]), color, 2, cv2.LINE_AA)
    h, w = shape
    pixels = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
    return corners, texture, pixels


def render(planes, center, rotation):
    image = np.full(SIZE[::-1], 45, np.uint8)
    for corners, texture, pixels in planes:
        camera = (corners-center) @ rotation.as_matrix() @ BODY_FROM_CAMERA
        projected = camera @ K.T
        uv = (projected[:, :2] / projected[:, 2:]).astype(np.float32)
        transform = cv2.getPerspectiveTransform(pixels, uv)
        cv2.warpPerspective(texture, transform, SIZE, image, flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_TRANSPARENT)
    return image


def generate(directory):
    images = directory / "images"
    images.mkdir(parents=True)
    rng = np.random.default_rng(2160)
    geometry = [(8, 0, 1.2, 24, 16), (5.2, 1.9, 1.35, 1.8, 3.2),
                (5.6, -2.1, 1.3, 1.6, 2.5), (3.9, -.28, 1.62, .85, 1.18),
                (3.4, 1.7, .75, .48, 1.5), (3.2, -1.5, 1.95, .54, 1.25)]
    planes = [textured_plane(rng, *spec) for spec in geometry]
    truth, times, imu = [], [], []
    for frame in range(601):
        t = frame/20
        center, rotation = position(t), orientation(t)
        image = render(planes, center, rotation)
        cv2.imwrite(str(images/f"{frame:06}.png"), image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        times.append([frame, 1000+t, 1.])
        truth.append([1000+t, *center, *rotation.as_quat()])
    dt = .0002
    for sample in range(6001):
        t = sample/200
        acceleration = (position(t+dt) - 2*position(t) + position(t-dt)) / dt**2
        gyro = (orientation(t-dt).inv() * orientation(t+dt)).as_rotvec() / (2*dt)
        specific_force = orientation(t).inv().apply(acceleration + [0, 0, 9.81007])
        imu.append([1000+t, *gyro, *specific_force])
    np.savetxt(directory/"truth-body.txt", truth, fmt="%.9f")
    np.savetxt(directory/"times.txt", times, fmt=["%d", "%.6f", "%.3f"])
    np.savetxt(directory/"imu.csv", imu, delimiter=",", fmt="%.9f")
    (directory/"camera.txt").write_text("Pinhole 440 440 319.5 239.5 0\n640 480\ncrop\n640 480\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="new output directory")
    generate(parser.parse_args().directory)
