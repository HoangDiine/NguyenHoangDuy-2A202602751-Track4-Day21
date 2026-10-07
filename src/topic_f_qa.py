"""Compare LiDAR-supported 2D boxes with KITTI image labels (Topic F)."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from starter.datasets import dataset_type, list_frames, load_frame
from starter.kitti_io import KittiObject
from starter.projection import box3d_corners_cam, cam_to_image, velo_to_cam


DEPTH_BINS = ("<20m", "20-40m", ">=40m")
OCCLUSION_LEVELS = (0, 1, 2)
METHODS = (
    ("projected_corners", "Projected 3D corners", "#2878b5"),
    ("lidar_points", "LiDAR points in 3D box", "#ed8b23"),
)


def depth_bin(depth_m: float) -> str:
    if depth_m < 20.0:
        return DEPTH_BINS[0]
    if depth_m < 40.0:
        return DEPTH_BINS[1]
    return DEPTH_BINS[2]


def bbox_from_uv(uv: np.ndarray) -> np.ndarray | None:
    """Return the visible min/max box for image points, or None when empty."""
    uv = np.asarray(uv, dtype=np.float64)
    if uv.ndim != 2 or uv.shape[1] != 2:
        raise ValueError("uv phải có shape (N, 2)")
    if uv.shape[0] == 0:
        return None
    finite_uv = uv[np.isfinite(uv).all(axis=1)]
    if finite_uv.shape[0] == 0:
        return None
    return np.array([
        finite_uv[:, 0].min(), finite_uv[:, 1].min(),
        finite_uv[:, 0].max(), finite_uv[:, 1].max(),
    ], dtype=np.float64)


def box_iou(box_a: np.ndarray | None, box_b: np.ndarray) -> float:
    """Continuous-coordinate 2D IoU; empty or zero-area predictions score 0."""
    if box_a is None:
        return 0.0
    a = np.asarray(box_a, dtype=np.float64)
    b = np.asarray(box_b, dtype=np.float64)
    if a.shape != (4,) or b.shape != (4,):
        raise ValueError("Mỗi bbox phải có 4 giá trị x1, y1, x2, y2")

    intersection_w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    intersection_h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = intersection_w * intersection_h
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - intersection
    return float(intersection / union) if union > 0.0 else 0.0


def points_in_object_box(points_cam: np.ndarray, obj: KittiObject) -> np.ndarray:
    """Select finite camera-frame points inside the oriented KITTI 3D box."""
    points_cam = np.asarray(points_cam, dtype=np.float64)
    h, w, length = np.asarray(obj.dimensions, dtype=np.float64)
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    rotation_y = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    valid = np.isfinite(points_cam).all(axis=1) & (points_cam[:, 2] > 0.1)
    candidate_points = points_cam[valid]
    if candidate_points.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)

    local = (candidate_points - np.asarray(obj.location, dtype=np.float64)) @ rotation_y
    inside = (
        (np.abs(local[:, 0]) <= length / 2.0)
        & (local[:, 1] >= -h)
        & (local[:, 1] <= 0.0)
        & (np.abs(local[:, 2]) <= w / 2.0)
    )
    return candidate_points[inside]


def projected_corner_box(obj: KittiObject, calib, image_shape) -> np.ndarray | None:
    corners = box3d_corners_cam(obj)
    uv, _, _ = cam_to_image(corners, calib.P2, image_shape)
    return bbox_from_uv(uv)


def lidar_point_box(points_in_box: np.ndarray, calib, image_shape):
    uv, _, _ = cam_to_image(points_in_box, calib.P2, image_shape)
    return bbox_from_uv(uv), int(points_in_box.shape[0]), int(uv.shape[0])


def box_has_area(box: np.ndarray | None) -> bool:
    return bool(box is not None and box[2] > box[0] and box[3] > box[1])


def box_fields(prefix: str, box: np.ndarray | None) -> dict[str, float | str]:
    names = ("x1", "y1", "x2", "y2")
    if box is None:
        return {f"{prefix}_{name}": "" for name in names}
    return {f"{prefix}_{name}": float(value) for name, value in zip(names, box)}


def analyze_dataset(data_root: str | Path) -> tuple[list[dict], list[str]]:
    if dataset_type(data_root) != "kitti":
        raise ValueError("Topic F script này cần KITTI có nhãn 2D và 3D độc lập")

    frame_ids = list_frames(data_root)
    rows = []
    for frame_id in frame_ids:
        frame = load_frame(data_root, frame_id)
        points_cam = velo_to_cam(frame["points"][:, :3], frame["calib"])
        car_index = 0
        for obj in frame["labels"]:
            if obj.type != "Car":
                continue

            gt_box = np.asarray(obj.bbox, dtype=np.float64)
            corner_box = projected_corner_box(obj, frame["calib"], frame["image"].shape)
            points_in_box = points_in_object_box(points_cam, obj)
            point_box, point_count_3d, point_count_visible = lidar_point_box(
                points_in_box, frame["calib"], frame["image"].shape
            )
            depth_m = float(obj.location[2])
            is_clean = obj.truncated <= 0.1 and obj.occluded <= 1

            row = {
                "frame_id": frame_id,
                "car_index": car_index,
                "depth_m": depth_m,
                "depth_bin": depth_bin(depth_m),
                "truncated": float(obj.truncated),
                "occluded": int(obj.occluded),
                "clean_subset": is_clean,
                "points_in_3d_box": point_count_3d,
                "points_projected_in_image": point_count_visible,
                "projected_corners_iou": box_iou(corner_box, gt_box),
                "projected_corners_has_area": box_has_area(corner_box),
                "lidar_points_iou": box_iou(point_box, gt_box),
                "lidar_points_has_area": box_has_area(point_box),
            }
            row.update(box_fields("gt", gt_box))
            row.update(box_fields("corners", corner_box))
            row.update(box_fields("lidar", point_box))
            rows.append(row)
            car_index += 1

    if not rows:
        raise ValueError(f"Không tìm thấy nhãn Car trong {data_root}")
    return rows, frame_ids


def build_summary(rows: list[dict]) -> list[dict]:
    summary = []
    subsets = (("clean", lambda row: row["clean_subset"]), ("all", lambda row: True))
    for subset_name, include in subsets:
        for bin_name in DEPTH_BINS:
            group = [row for row in rows if include(row) and row["depth_bin"] == bin_name]
            point_counts = [row["points_in_3d_box"] for row in group]
            for method_key, method_label, _ in METHODS:
                iou_key = f"{method_key}_iou"
                area_key = f"{method_key}_has_area"
                ious = [row[iou_key] for row in group]
                summary.append({
                    "subset": subset_name,
                    "depth_bin": bin_name,
                    "method": method_key,
                    "method_label": method_label,
                    "n_objects": len(group),
                    "n_boxes_with_area": sum(bool(row[area_key]) for row in group),
                    "median_iou": float(np.median(ious)) if ious else "",
                    "median_points_in_3d_box": float(np.median(point_counts)) if point_counts else "",
                })
    return summary


def build_occlusion_summary(rows: list[dict]) -> list[dict]:
    """Compare box IoU by KITTI occlusion level, holding class and truncation fixed."""
    eligible = [
        row for row in rows
        if float(row["truncated"]) <= 0.1 and int(row["occluded"]) in OCCLUSION_LEVELS
    ]
    summary = []
    for depth_name in ("all", *DEPTH_BINS):
        for occluded in OCCLUSION_LEVELS:
            group = [
                row for row in eligible
                if int(row["occluded"]) == occluded
                and (depth_name == "all" or row["depth_bin"] == depth_name)
            ]
            point_counts = [row["points_in_3d_box"] for row in group]
            depths = [row["depth_m"] for row in group]
            for method_key, method_label, _ in METHODS:
                iou_key = f"{method_key}_iou"
                area_key = f"{method_key}_has_area"
                ious = [row[iou_key] for row in group]
                summary.append({
                    "depth_bin": depth_name,
                    "truncated_max": 0.1,
                    "occluded": occluded,
                    "method": method_key,
                    "method_label": method_label,
                    "n_objects": len(group),
                    "n_boxes_with_area": sum(bool(row[area_key]) for row in group),
                    "median_iou": float(np.median(ious)) if ious else "",
                    "median_points_in_3d_box": float(np.median(point_counts)) if point_counts else "",
                    "median_depth_m": float(np.median(depths)) if depths else "",
                })
    return summary


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError(f"Không có dòng để ghi vào {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(summary: list[dict], output_path: Path) -> None:
    lookup = {(row["subset"], row["depth_bin"], row["method"]): row for row in summary}
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    x = np.arange(len(DEPTH_BINS))
    width = 0.34

    for ax, subset_name, title in zip(axes, ("clean", "all"), ("Clean subset", "All labeled Cars")):
        for method_index, (method_key, method_label, color) in enumerate(METHODS):
            values = []
            for bin_name in DEPTH_BINS:
                entry = lookup[(subset_name, bin_name, method_key)]
                values.append(float(entry["median_iou"]) if entry["n_objects"] else np.nan)
            positions = x + (method_index - 0.5) * width
            bars = ax.bar(positions, values, width, color=color, label=method_label)
            for bar, value in zip(bars, values):
                if np.isfinite(value):
                    ax.text(bar.get_x() + bar.get_width() / 2, value + 0.025,
                            f"{value:.2f}", ha="center", va="bottom", fontsize=8)

        counts = [lookup[(subset_name, bin_name, METHODS[0][0])]["n_objects"] for bin_name in DEPTH_BINS]
        ax.set_xticks(x, [f"{bin_name}\nn={count}" for bin_name, count in zip(DEPTH_BINS, counts)])
        ax.set_title(title)
        ax.set_ylim(0.0, 1.08)
        ax.grid(axis="y", alpha=0.25)
        ax.set_axisbelow(True)

    axes[0].set_ylabel("Median IoU against KITTI 2D label")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, frameon=False)
    fig.suptitle("Topic F: 2D box consistency by camera depth")
    fig.tight_layout(rect=(0, 0.1, 1, 0.93))
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_occlusion_summary(summary: list[dict], output_path: Path) -> None:
    lookup = {
        (int(row["occluded"]), row["method"]): row
        for row in summary if row["depth_bin"] == "all"
    }
    x = np.arange(len(OCCLUSION_LEVELS))
    fig, ax = plt.subplots(figsize=(7, 4.5))

    for method_key, method_label, color in METHODS:
        values = [float(lookup[(level, method_key)]["median_iou"]) for level in OCCLUSION_LEVELS]
        ax.plot(x, values, marker="o", linewidth=2, color=color, label=method_label)
        for x_pos, value in zip(x, values):
            ax.annotate(f"{value:.3f}", (x_pos, value), xytext=(0, 8),
                        textcoords="offset points", ha="center", fontsize=8)

    counts = [int(lookup[(level, METHODS[0][0])]["n_objects"]) for level in OCCLUSION_LEVELS]
    ax.set_xticks(x, [f"{level}\nn={count}" for level, count in zip(OCCLUSION_LEVELS, counts)])
    ax.set_xlabel("KITTI occlusion level (0 = visible, 2 = heavily occluded)")
    ax.set_ylabel("Median IoU against KITTI 2D label")
    ax.set_ylim(0.0, 1.05)
    ax.set_title("Topic F: suggested 2D box quality by occlusion")
    ax.grid(axis="y", alpha=0.25)
    ax.set_axisbelow(True)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def add_box(ax, box: np.ndarray | None, color: str, label: str, linewidth: float = 2.0) -> None:
    if box is None:
        return
    x1, y1, x2, y2 = (float(value) for value in box)
    if box_has_area(box):
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                               edgecolor=color, linewidth=linewidth, label=label))
    else:
        ax.scatter([(x1 + x2) / 2], [(y1 + y2) / 2], marker="x", s=100,
                   linewidths=2.5, color=color, label=f"{label} (zero area)")


def save_object_figure(data_root: str | Path, frame_id: str, car_index: int,
                       output_path: Path, title_prefix: str) -> dict:
    frame = load_frame(data_root, frame_id)
    cars = [obj for obj in frame["labels"] if obj.type == "Car"]
    if car_index < 0 or car_index >= len(cars):
        raise ValueError(f"Frame {frame_id} chỉ có {len(cars)} nhãn Car; car-index={car_index} không hợp lệ")

    obj = cars[car_index]
    points_cam = velo_to_cam(frame["points"][:, :3], frame["calib"])
    points_in_box = points_in_object_box(points_cam, obj)
    point_box, point_count, _ = lidar_point_box(points_in_box, frame["calib"], frame["image"].shape)
    corner_box = projected_corner_box(obj, frame["calib"], frame["image"].shape)
    point_uv, _, _ = cam_to_image(points_in_box, frame["calib"].P2, frame["image"].shape)

    if point_uv.shape[0] > 500:
        sample_indices = np.linspace(0, point_uv.shape[0] - 1, 500, dtype=int)
        point_uv = point_uv[sample_indices]

    rgb = frame["image"][:, :, ::-1]
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), gridspec_kw={"width_ratios": [1.8, 1]})
    for ax in axes:
        ax.imshow(rgb)
        ax.scatter(point_uv[:, 0], point_uv[:, 1], s=7, color="#29c5d8", alpha=0.8,
                   label="LiDAR points inside 3D box")
        add_box(ax, np.asarray(obj.bbox), "#27a844", "KITTI 2D label")
        add_box(ax, corner_box, "#2878b5", "Projected 3D corners")
        add_box(ax, point_box, "#ed8b23", "LiDAR point box")
        ax.set_xlim(0, rgb.shape[1])
        ax.set_ylim(rgb.shape[0], 0)
        ax.axis("off")

    gt_box = np.asarray(obj.bbox, dtype=np.float64)
    pad_x = max(30.0, (gt_box[2] - gt_box[0]) * 0.8)
    pad_y = max(30.0, (gt_box[3] - gt_box[1]) * 0.8)
    axes[1].set_xlim(max(0.0, gt_box[0] - pad_x), min(rgb.shape[1], gt_box[2] + pad_x))
    axes[1].set_ylim(min(rgb.shape[0], gt_box[3] + pad_y), max(0.0, gt_box[1] - pad_y))
    axes[0].set_title("Full image")
    axes[1].set_title("Object crop")

    point_iou = box_iou(point_box, gt_box)
    corner_iou = box_iou(corner_box, gt_box)
    fig.suptitle(
        f"{title_prefix}: frame {frame_id}, Car #{car_index}, z={obj.location[2]:.1f} m, "
        f"points in 3D box={point_count}; IoU corners={corner_iou:.3f}, points={point_iou:.3f}"
    )
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False, fontsize=9)
    fig.tight_layout(rect=(0, 0.12, 1, 0.91))
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    return {
        "frame_id": frame_id,
        "car_index": car_index,
        "depth_m": float(obj.location[2]),
        "points_in_3d_box": point_count,
        "points_iou": point_iou,
        "corners_iou": corner_iou,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="So sánh box 2D gợi ý từ 3D corners và điểm LiDAR trên KITTI mini."
    )
    parser.add_argument("--data-root", default="data/kitti_mini", help="thư mục KITTI có training/label_2")
    parser.add_argument("--out-dir", default="results", help="thư mục lưu CSV và figures")
    parser.add_argument("--demo-frame", default="000004", help="frame tạo ảnh overlay demo")
    parser.add_argument("--demo-car-index", type=int, default=0, help="chỉ số Car trong frame demo, bắt đầu từ 0")
    parser.add_argument("--failure-frame", default="000009", help="frame dùng cho ảnh failure case")
    parser.add_argument("--failure-car-index", type=int, default=2,
                        help="chỉ số Car trong frame failure, bắt đầu từ 0")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    figures_dir = out_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    rows, frame_ids = analyze_dataset(args.data_root)
    summary = build_summary(rows)
    occlusion_summary = build_occlusion_summary(rows)
    objects_csv = out_dir / "topic_f_object_iou.csv"
    summary_csv = out_dir / "topic_f_iou_by_distance.csv"
    figure_path = figures_dir / "topic_f_iou_by_distance.png"
    occlusion_csv = out_dir / "topic_f_iou_by_occlusion.csv"
    occlusion_figure = figures_dir / "topic_f_iou_by_occlusion.png"
    write_csv(objects_csv, rows)
    write_csv(summary_csv, summary)
    write_csv(occlusion_csv, occlusion_summary)
    plot_summary(summary, figure_path)
    plot_occlusion_summary(occlusion_summary, occlusion_figure)

    if args.demo_frame not in frame_ids:
        raise ValueError(f"Không tìm thấy demo frame {args.demo_frame} trong {args.data_root}")
    if args.failure_frame not in frame_ids:
        raise ValueError(f"Không tìm thấy failure frame {args.failure_frame} trong {args.data_root}")

    demo_path = figures_dir / f"topic_f_demo_{args.demo_frame}.png"
    failure_path = figures_dir / f"fail_01_sparse_lidar_car_{args.failure_frame}.png"
    save_object_figure(args.data_root, args.demo_frame, args.demo_car_index, demo_path, "Demo")
    failure = save_object_figure(
        args.data_root, args.failure_frame, args.failure_car_index, failure_path, "Failure case"
    )

    clean_count = sum(bool(row["clean_subset"]) for row in rows)
    print(f"frames={len(frame_ids)} cars={len(rows)} clean_cars={clean_count}")
    for row in summary:
        if row["subset"] != "clean":
            continue
        print(
            f"clean {row['depth_bin']:>5} {row['method']}: "
            f"median_IoU={row['median_iou']:.3f} n={row['n_objects']} "
            f"boxes_with_area={row['n_boxes_with_area']}"
        )
    print(f"objects_csv={objects_csv}")
    print(f"summary_csv={summary_csv}")
    print(f"iou_plot={figure_path}")
    for row in occlusion_summary:
        if row["depth_bin"] != "all":
            continue
        print(
            f"truncated<=0.1 occluded={row['occluded']} {row['method']}: "
            f"median_IoU={row['median_iou']:.3f} n={row['n_objects']}"
        )
    print(f"occlusion_csv={occlusion_csv}")
    print(f"occlusion_plot={occlusion_figure}")
    print(f"demo_image={demo_path}")
    print(
        f"failure_image={failure_path} points={failure['points_in_3d_box']} "
        f"lidar_iou={failure['points_iou']:.3f}"
    )


if __name__ == "__main__":
    main()
