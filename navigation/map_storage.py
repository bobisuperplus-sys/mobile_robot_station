# -*- coding: utf-8 -*-
"""
工业级地图存盘与持久化存储管理系统 (Map Storage & Persistence Manager)

核心特性：
1. 3D 稠密空间点云存储：标准 ASCII PLY 格式，完全兼容 CloudCompare、MeshLab 等工业点云软件；
2. 2.5D 高程与坡度矩阵存储：采用极速高压缩比 NumPy 归档 (.npz)，支持毫秒级无损反序列化还原；
3. 可通行性格网直观导出：输出标准 8-bit PNG 图像，清晰标识安全路面 (0)、缓坡梯度 (1~120) 与致命台阶/障碍物 (254)；
4. 标准化元数据配置：YAML 格式规范记录地图尺寸、物理分辨率、世界坐标原点、机器人几何半径与最大越障坡度；
5. 地图资产管理：提供地图列表检索、完整性校验、一键加载与安全删除。
"""

import datetime
import os
import shutil
import time
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from navigation.elevation_map import ElevationMap2D


class MapStorage:
    """工业移动机器人地图持久化存储与加载管理器"""

    METADATA_FILENAME = "metadata.yaml"
    ELEVATION_DATA_FILENAME = "elevation_data.npz"
    POINT_CLOUD_FILENAME = "point_cloud.ply"
    TRAVERSABILITY_IMG_FILENAME = "traversability.png"
    ELEVATION_TEXTURE_FILENAME = "elevation_texture.png"
    ELEVATION_HEATMAP_FILENAME = "elevation_heatmap.png"

    @classmethod
    def save_map(
        cls,
        map_name: str,
        elevation_map: ElevationMap2D,
        point_cloud: Optional[np.ndarray] = None,
        output_dir: str = "maps",
        description: str = "3D LIO-SLAM & 2.5D Elevation Map",
    ) -> Dict[str, str]:
        """
        持久化保存完整的地图包，包含 3D 点云、2.5D 高程矩阵、可通行性图像与 YAML 元数据

        参数:
            map_name: 地图唯一标识名称 (如 urban_world_v1)
            elevation_map: 构造完毕的 ElevationMap2D 实例
            point_cloud: 原始三维激光点云数组 (N, 3)，可选
            output_dir: 地图根目录 (相对工程根目录或指定路径)
            description: 地图说明文本
        返回:
            保存生成的各文件相对路径字典
        """
        clean_name = map_name.strip().replace(" ", "_")
        target_dir = os.path.join(output_dir, clean_name)
        os.makedirs(target_dir, exist_ok=True)

        saved_files: Dict[str, str] = {}

        # 1. 保存 2.5D 核心数组矩阵 (.npz)
        npz_path = os.path.join(target_dir, cls.ELEVATION_DATA_FILENAME)
        np.savez_compressed(
            npz_path,
            elevation_array=elevation_map.elevation_array,
            slope_deg_array=elevation_map.slope_deg_array,
            step_height_array=elevation_map.step_height_array,
            cost_array=elevation_map.cost_array,
        )
        saved_files["elevation_data"] = npz_path

        # 2. 保存 3D 点云 (标准 ASCII PLY 格式)
        num_points = 0
        if point_cloud is not None and len(point_cloud) > 0:
            ply_path = os.path.join(target_dir, cls.POINT_CLOUD_FILENAME)
            cls._write_ascii_ply(ply_path, point_cloud)
            saved_files["point_cloud"] = ply_path
            num_points = len(point_cloud)

        # 3. 渲染并导出可通行性格网直观图 (.png)
        png_path = os.path.join(target_dir, cls.TRAVERSABILITY_IMG_FILENAME)
        cls._render_traversability_image(png_path, elevation_map)
        saved_files["traversability_img"] = png_path

        # 4. 导出 2.5D 高程矩阵 RGBA 贴图 (.png)
        tex_path = os.path.join(target_dir, cls.ELEVATION_TEXTURE_FILENAME)
        cls._render_elevation_texture(tex_path, elevation_map)
        saved_files["elevation_texture"] = tex_path

        # 5. 导出专业 2.5D 高程标尺热力图 (.png)
        heat_path = os.path.join(target_dir, cls.ELEVATION_HEATMAP_FILENAME)
        cls._render_elevation_heatmap(heat_path, elevation_map)
        saved_files["elevation_heatmap"] = heat_path

        # 6. 生成标准化元数据配置文件 (.yaml)
        now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        metadata: Dict[str, Any] = {
            "format_version": "1.0.0",
            "map_name": clean_name,
            "created_at": now_str,
            "description": description,
            "geometry": {
                "resolution": float(elevation_map.resolution),
                "size_x": float(elevation_map.size_x),
                "size_y": float(elevation_map.size_y),
                "origin_x": float(elevation_map.origin_x),
                "origin_y": float(elevation_map.origin_y),
                "grid_nx": int(elevation_map.nx),
                "grid_ny": int(elevation_map.ny),
            },
            "parameters": {
                "robot_radius": float(elevation_map.robot_radius),
                "inflation_radius": float(elevation_map.inflation_radius),
                "decay_factor": float(elevation_map.decay_factor),
                "max_slope_deg": float(elevation_map.max_slope_deg),
                "max_step_height": float(elevation_map.max_step_height),
                "default_ground_z": float(elevation_map.default_ground_z),
            },
            "statistics": {
                "point_count": int(num_points),
                "min_elevation": float(np.min(elevation_map.elevation_array)),
                "max_elevation": float(np.max(elevation_map.elevation_array)),
                "mean_slope_deg": float(np.mean(elevation_map.slope_deg_array)),
                "lethal_obstacle_cells": int(np.sum(elevation_map.cost_array == ElevationMap2D.LETHAL_OBSTACLE)),
                "free_cells": int(np.sum(elevation_map.cost_array == ElevationMap2D.FREE_SPACE)),
            },
            "files": {
                "metadata": cls.METADATA_FILENAME,
                "elevation_data": cls.ELEVATION_DATA_FILENAME,
                "point_cloud": cls.POINT_CLOUD_FILENAME if num_points > 0 else None,
                "traversability_image": cls.TRAVERSABILITY_IMG_FILENAME,
            },
        }

        yaml_path = os.path.join(target_dir, cls.METADATA_FILENAME)
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False, allow_unicode=True)
        saved_files["metadata"] = yaml_path

        return saved_files

    @classmethod
    def load_map(
        cls,
        map_path_or_name: str,
        maps_root: str = "maps",
    ) -> Tuple[ElevationMap2D, Optional[np.ndarray], Dict[str, Any]]:
        """
        从指定地图目录或名称加载完整的先验地图包

        参数:
            map_path_or_name: 地图名称 (如 urban_world_v1) 或完整物理目录
            maps_root: 地图存储根目录
        返回:
            (ElevationMap2D 实例, 原始点云数组或 None, 元数据字典)
        """
        target_dir = map_path_or_name
        if not os.path.isdir(target_dir):
            target_dir = os.path.join(maps_root, map_path_or_name)

        if not os.path.isdir(target_dir):
            raise FileNotFoundError(f"地图目录不存在: {target_dir}")

        yaml_path = os.path.join(target_dir, cls.METADATA_FILENAME)
        npz_path = os.path.join(target_dir, cls.ELEVATION_DATA_FILENAME)

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"缺少元数据配置文件: {yaml_path}")
        if not os.path.exists(npz_path):
            raise FileNotFoundError(f"缺少高程矩阵数据文件: {npz_path}")

        # 1. 读取元数据
        with open(yaml_path, "r", encoding="utf-8") as f:
            metadata = yaml.safe_load(f)

        geom = metadata.get("geometry", {})
        params = metadata.get("parameters", {})

        # 2. 实例化 ElevationMap2D
        elev_map = ElevationMap2D(
            resolution=geom.get("resolution", 0.05),
            size_x=geom.get("size_x", 24.0),
            size_y=geom.get("size_y", 24.0),
            origin_x=geom.get("origin_x", -12.0),
            origin_y=geom.get("origin_y", -12.0),
            robot_radius=params.get("robot_radius", 0.12),
            inflation_radius=params.get("inflation_radius", 0.35),
            decay_factor=params.get("decay_factor", 6.0),
            max_slope_deg=params.get("max_slope_deg", 16.0),
            max_step_height=params.get("max_step_height", 0.06),
            default_ground_z=params.get("default_ground_z", 0.0),
        )

        # 3. 反序列化数组
        with np.load(npz_path) as data:
            elev_map.elevation_array = np.array(data["elevation_array"], dtype=np.float32)
            elev_map.slope_deg_array = np.array(data["slope_deg_array"], dtype=np.float32)
            elev_map.step_height_array = np.array(data["step_height_array"], dtype=np.float32)
            elev_map.cost_array = np.array(data["cost_array"], dtype=np.uint8)

        # 4. 可选读取点云
        pts = None
        ply_path = os.path.join(target_dir, cls.POINT_CLOUD_FILENAME)
        if os.path.exists(ply_path):
            pts = cls._read_ascii_ply(ply_path)

        return elev_map, pts, metadata

    @classmethod
    def list_maps(cls, maps_root: str = "maps") -> List[Dict[str, Any]]:
        """列举存储库中所有可用的地图清单"""
        if not os.path.exists(maps_root):
            return []

        results: List[Dict[str, Any]] = []
        for entry in sorted(os.listdir(maps_root)):
            subdir = os.path.join(maps_root, entry)
            if not os.path.isdir(subdir):
                continue
            yaml_path = os.path.join(subdir, cls.METADATA_FILENAME)
            if not os.path.exists(yaml_path):
                continue

            try:
                with open(yaml_path, "r", encoding="utf-8") as f:
                    meta = yaml.safe_load(f)
                info = {
                    "name": meta.get("map_name", entry),
                    "created_at": meta.get("created_at", "Unknown"),
                    "description": meta.get("description", ""),
                    "size_x": meta.get("geometry", {}).get("size_x", 0.0),
                    "size_y": meta.get("geometry", {}).get("size_y", 0.0),
                    "resolution": meta.get("geometry", {}).get("resolution", 0.05),
                    "point_count": meta.get("statistics", {}).get("point_count", 0),
                    "directory": subdir,
                }
                results.append(info)
            except Exception:
                continue

        return results

    @classmethod
    def delete_map(cls, map_name: str, maps_root: str = "maps") -> bool:
        """从存储库中安全删除指定地图"""
        target_dir = os.path.join(maps_root, map_name.strip().replace(" ", "_"))
        if os.path.isdir(target_dir):
            shutil.rmtree(target_dir)
            return True
        return False

    @staticmethod
    def _write_ascii_ply(file_path: str, points: np.ndarray) -> None:
        """将三维点云高效写入标准 ASCII PLY 文件"""
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        n = len(pts)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write("comment Mobile Robot Station 3D SLAM Map\n")
            f.write(f"element vertex {n}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for i in range(n):
                f.write(f"{pts[i, 0]:.4f} {pts[i, 1]:.4f} {pts[i, 2]:.4f}\n")

    @staticmethod
    def _read_ascii_ply(file_path: str) -> np.ndarray:
        """读取标准 ASCII PLY 点云"""
        pts: List[List[float]] = []
        with open(file_path, "r", encoding="utf-8") as f:
            in_header = True
            for line in f:
                line = line.strip()
                if in_header:
                    if line == "end_header":
                        in_header = False
                    continue
                if line:
                    parts = line.split()
                    if len(parts) >= 3:
                        pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
        return np.array(pts, dtype=np.float32)

    @staticmethod
    def _render_traversability_image(file_path: str, elev_map: ElevationMap2D) -> None:
        """导出高分辨率可通行性格网诊断图"""
        fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
        fig.patch.set_facecolor("#0b0e14")
        ax.set_facecolor("#06080c")

        extent = [
            elev_map.origin_x,
            elev_map.origin_x + elev_map.size_x,
            elev_map.origin_y,
            elev_map.origin_y + elev_map.size_y,
        ]
        im = ax.imshow(
            elev_map.cost_array,
            origin="lower",
            extent=extent,
            cmap="viridis",
            vmin=0,
            vmax=255,
        )
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Traversability Cost (0:free, 1-120:slope, 254:lethal)", color="#b0b8c4", fontsize=8)
        cbar.ax.tick_params(colors="#8892a0", labelsize=7)

        ax.set_title(f"2.5D Traversability Grid ({elev_map.nx}x{elev_map.ny})", color="white", fontsize=10, pad=8)
        ax.tick_params(colors="#8892a0", labelsize=7)
        ax.grid(True, color="#1e232d", linestyle=":", alpha=0.5)

        plt.tight_layout()
        plt.savefig(file_path, facecolor=fig.get_facecolor(), edgecolor="none")
        plt.close()

    @staticmethod
    def _render_elevation_texture(file_path: str, elev_map: ElevationMap2D) -> None:
        """导出纯 2.5D 高程矩阵 RGBA 贴图，供上位机地面站无损投影使用"""
        import matplotlib.colors as mcolors
        from PIL import Image

        norm = mcolors.Normalize(vmin=0.0, vmax=0.45, clip=True)
        cmap = plt.get_cmap("terrain")
        rgba = cmap(norm(elev_map.elevation_array))
        rgba_img = (np.flipud(rgba) * 255).astype(np.uint8)
        img = Image.fromarray(rgba_img)
        img.save(file_path)

    @staticmethod
    def _render_elevation_heatmap(file_path: str, elev_map: ElevationMap2D) -> None:
        """导出专业 2.5D 高程热力图与科学色彩标尺 (DEM Heatmap with Colorbar)"""
        fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
        fig.patch.set_facecolor("#0b0e14")
        ax.set_facecolor("#06080c")

        extent = [
            elev_map.origin_x,
            elev_map.origin_x + elev_map.size_x,
            elev_map.origin_y,
            elev_map.origin_y + elev_map.size_y,
        ]
        im = ax.imshow(
            elev_map.elevation_array,
            origin="lower",
            extent=extent,
            cmap="terrain",
            vmin=0.0,
            vmax=0.45,
            alpha=0.95,
        )
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Elevation Z (m)", color="#b0b8c4", fontsize=9)
        cbar.ax.tick_params(colors="#8892a0", labelsize=8)

        ax.set_title(f"2.5D Elevation Map ({elev_map.nx}x{elev_map.ny})", color="white", fontsize=10, pad=8)
        ax.tick_params(colors="#8892a0", labelsize=8)
        ax.set_xlabel("World X (m)", color="#b0b8c4", fontsize=8)
        ax.set_ylabel("World Y (m)", color="#b0b8c4", fontsize=8)
        ax.grid(True, color="#1e232d", linestyle=":", alpha=0.5)

        plt.tight_layout()
        plt.savefig(file_path, facecolor=fig.get_facecolor(), edgecolor="none")
        plt.close()

