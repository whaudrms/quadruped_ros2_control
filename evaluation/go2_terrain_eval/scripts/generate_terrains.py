#!/usr/bin/env python3

import argparse
import importlib.util
import os
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


ROOT = Path("/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval")
TERRAIN_TOOL_DIR = Path("/home/ho/unitree_mujoco/terrain_tool")
GO2_DIR = Path("/home/ho/unitree_mujoco/unitree_robots/go2")
ASSETS_DIR = ROOT / "assets"


def load_catalog():
    with open(ROOT / "configs" / "terrains.yaml", "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["terrains"]


def load_generator_module():
    ensure_noise_module()
    module_path = TERRAIN_TOOL_DIR / "terrain_generator.py"
    spec = importlib.util.spec_from_file_location("terrain_generator_eval", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def ensure_noise_module():
    if "noise" in sys.modules:
        return
    try:
        import noise  # noqa: F401
        return
    except ModuleNotFoundError:
        pass

    fallback = types.ModuleType("noise")

    def pnoise2(x, y, octaves=1, persistence=0.5, lacunarity=2.0):
        value = 0.0
        amplitude = 1.0
        frequency = 1.0
        norm = 0.0
        for octave in range(max(1, int(octaves))):
            phase = octave * 17.0
            layer = math.sin((x + phase) * 1.7 * frequency) * math.cos((y - phase) * 1.3 * frequency)
            value += amplitude * layer
            norm += amplitude
            amplitude *= persistence
            frequency *= lacunarity
        value = value / norm if norm > 0.0 else 0.0
        return max(-1.0, min(1.0, value))

    import math

    fallback.pnoise2 = pnoise2
    sys.modules["noise"] = fallback


def instantiate_generator(module):
    cwd = os.getcwd()
    os.chdir(TERRAIN_TOOL_DIR)
    try:
        tg = module.TerrainGenerator()
    finally:
        os.chdir(cwd)
    return tg


def set_output_scene(tg, output_scene: Path):
    output_scene.parent.mkdir(parents=True, exist_ok=True)
    tg.scene.write(output_scene)


def remove_existing_generated_asset(output_image_name: str):
    asset_path = GO2_DIR / output_image_name
    if asset_path.exists():
        asset_path.unlink()


def box_vertical_half_extent(module, size, euler):
    half = 0.5 * np.array(size, dtype=np.float64)
    rot = module.euler_to_rot(euler[0], euler[1], euler[2])
    return float(np.abs(rot[2, :]) @ half)


def add_box_ground_aligned(tg, module, position, euler, size, ground_z=0.0):
    aligned_position = np.array(position, dtype=np.float64)
    aligned_position[2] = ground_z + box_vertical_half_extent(module, size, euler)
    tg.AddBox(position=aligned_position.tolist(), euler=euler, size=size)


def add_rough_ground_aligned(tg, module, **params):
    init_pos = np.array(params.get("init_pos", [1.0, 0.0, 0.0]), dtype=np.float64)
    euler = np.array(params.get("euler", [0.0, 0.0, 0.0]), dtype=np.float64)
    nums = params.get("nums", [10, 10])
    box_size = np.array(params.get("box_size", [0.5, 0.5, 0.5]), dtype=np.float64)
    box_euler = np.array(params.get("box_euler", [0.0, 0.0, 0.0]), dtype=np.float64)
    separation = np.array(params.get("separation", [0.2, 0.2]), dtype=np.float64)
    box_size_rand = np.array(params.get("box_size_rand", [0.05, 0.05, 0.05]), dtype=np.float64)
    box_euler_rand = np.array(params.get("box_euler_rand", [0.2, 0.2, 0.2]), dtype=np.float64)
    separation_rand = np.array(params.get("separation_rand", [0.05, 0.05]), dtype=np.float64)

    local_xy = np.array([0.0, 0.0], dtype=np.float64)
    new_separation = separation + separation_rand * np.random.uniform(-1.0, 1.0, 2)
    for _ in range(nums[0]):
        local_xy[0] += new_separation[0]
        local_xy[1] = 0.0
        for _ in range(nums[1]):
            new_box_size = box_size + box_size_rand * np.random.uniform(-1.0, 1.0, 3)
            new_box_euler = box_euler + box_euler_rand * np.random.uniform(-1.0, 1.0, 3)
            new_separation = separation + separation_rand * np.random.uniform(-1.0, 1.0, 2)

            local_xy[1] += new_separation[1]
            center_z = box_vertical_half_extent(module, new_box_size, new_box_euler)
            local_pos = np.array([local_xy[0], local_xy[1], center_z], dtype=np.float64)
            pos = module.rot3d(local_pos, euler) + init_pos
            tg.AddBox(pos.tolist(), new_box_euler.tolist(), new_box_size.tolist())


def generate_slope_perceptive_image(output_path: Path, width: int = 160, height: int = 128):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image = np.full((height, width), 35, dtype=np.uint8)
    start = int(width * 0.65)
    end = int(width * 0.92)
    for x in range(start, end):
        alpha = (x - start) / max(1, (end - start))
        image[:, x] = int(35 + alpha * 130)
    image[:, end:] = 165
    cv2.imwrite(str(output_path), image)


def generate_rough_ground_perceptive_image(output_path: Path, width: int = 160, height: int = 128, seed: int = 7):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    image = np.full((height, width), 40, dtype=np.uint8)
    x0 = int(width * 0.62)
    block_w = 10
    block_h = 10
    for y in range(0, height, block_h):
        for x in range(x0, width, block_w):
            value = int(rng.integers(45, 105))
            image[y:y + block_h, x:x + block_w] = value
    image = cv2.GaussianBlur(image, (5, 5), 0)
    cv2.imwrite(str(output_path), image)


def add_flat_prefix_to_hfield(image_path: Path, prefix_ratio: float = 0.38):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(str(image_path))
    width = image.shape[1]
    prefix_cols = int(width * prefix_ratio)
    flat_value = int(np.percentile(image[:, : max(1, width // 8)], 15))
    image[:, :prefix_cols] = flat_value
    image = cv2.GaussianBlur(image, (5, 5), 0)
    cv2.imwrite(str(image_path), image)


def generate_one(name: str, cfg: dict):
    module = load_generator_module()
    tg = instantiate_generator(module)
    scene_file = Path(cfg["scene_file"])
    terrain_type = cfg["type"]
    params = cfg.get("parameters", {})

    if terrain_type == "flat":
        pass
    elif terrain_type == "box":
        add_box_ground_aligned(tg, module, **params)
    elif terrain_type == "slope":
        add_box_ground_aligned(tg, module, **params)
        if cfg.get("perceptive_supported"):
            generate_slope_perceptive_image(Path(cfg["perceptive_image"]))
    elif terrain_type == "stairs":
        tg.AddStairs(**params)
    elif terrain_type == "suspend_stairs":
        tg.AddSuspendStairs(**params)
    elif terrain_type == "rough_ground":
        add_rough_ground_aligned(tg, module, **params)
        if cfg.get("perceptive_supported"):
            generate_rough_ground_perceptive_image(Path(cfg["perceptive_image"]))
    elif terrain_type == "perlin_hfield":
        output_image = params["output_hfield_image"]
        remove_existing_generated_asset(output_image)
        cwd = os.getcwd()
        os.chdir(TERRAIN_TOOL_DIR)
        try:
          tg.AddPerlinHeighField(**params)
        finally:
          os.chdir(cwd)
        if cfg.get("perceptive_supported"):
            add_flat_prefix_to_hfield(GO2_DIR / output_image)
    elif terrain_type == "image_hfield":
        output_image = params["output_hfield_image"]
        remove_existing_generated_asset(output_image)
        cwd = os.getcwd()
        os.chdir(TERRAIN_TOOL_DIR)
        try:
          tg.AddHeighFieldFromImage(**params)
        finally:
          os.chdir(cwd)
    elif terrain_type == "complex_course":
        tg.AddBox(position=[1.5, 0.0, 0.1], euler=[0.0, 0.0, 0.0], size=[1.0, 1.5, 0.2])
        tg.AddGeometry(position=[1.5, 0.0, 0.25], euler=[0.0, 0.0, 0.0], size=[1.0, 0.5, 0.5], geo_type="cylinder")
        tg.AddBox(position=[2.0, 2.0, 0.5], euler=[0.0, -0.5, 0.0], size=[3.0, 1.5, 0.1])
        tg.AddStairs(init_pos=[1.0, 4.0, 0.0], yaw=0.0)
        tg.AddSuspendStairs(init_pos=[1.0, 6.0, 0.0], yaw=0.0)
        tg.AddRoughGround(init_pos=[-2.5, 5.0, 0.0], euler=[0.0, 0.0, 0.0], nums=[10, 8])
        cwd = os.getcwd()
        os.chdir(TERRAIN_TOOL_DIR)
        try:
            tg.AddPerlinHeighField(position=[-1.5, 4.0, 0.0], size=[2.0, 1.5], output_hfield_image="eval_complex_perlin.png")
            tg.AddHeighFieldFromImage(
                position=[-1.5, 2.0, 0.0],
                euler=[0.0, 0.0, -1.57],
                size=[2.0, 2.0],
                input_img=str(GO2_DIR / "unitree_hfield.png"),
                image_scale=[1.0, 1.0],
                output_hfield_image="eval_complex_image.png",
            )
        finally:
            os.chdir(cwd)
    else:
        raise ValueError(f"Unsupported terrain type: {terrain_type}")

    set_output_scene(tg, scene_file)
    return scene_file


def main():
    parser = argparse.ArgumentParser(description="Generate GO2 evaluation terrain scenes.")
    parser.add_argument("--terrain", help="Terrain name from terrains.yaml")
    parser.add_argument("--all", action="store_true", help="Generate all terrains")
    args = parser.parse_args()

    catalog = load_catalog()
    names = list(catalog.keys()) if args.all else [args.terrain]
    if not names or names == [None]:
        parser.error("Specify --terrain NAME or --all")

    for name in names:
        if name not in catalog:
            raise KeyError(f"Unknown terrain: {name}")
        scene = generate_one(name, catalog[name])
        print(f"[generated] {name}: {scene}")


if __name__ == "__main__":
    main()
