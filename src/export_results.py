import re
from pathlib import Path
import pandas as pd
import pyvista as pv

def save_data(pointcloud: pv.DataSet, output_dir: Path | str, name: str = "control_volume_points") -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Determine trial number based on existing files in output_dir
    pattern = re.compile(rf"^{re.escape(name)}_(\d+)\.(csv|vtk)$")
    trial_numbers = []

    for file in output_dir.iterdir():
        match = pattern.match(file.name)
        if match:
            trial_numbers.append(int(match.group(1)))

    trial_num = max(trial_numbers, default=0) + 1

    # 2. Save as VTK file
    vtk_path = output_dir / f"{name}_{trial_num}.vtk"
    pointcloud.save(str(vtk_path))
    print(f"[DataSaver] Saved VTK: {vtk_path}")

    # 3. Extract XYZ coordinates
    data = {
        'calculated_range': ((pointcloud.points[:, 0])**2 + (pointcloud.points[:, 1])**2 + (pointcloud.points[:, 2])**2)**0.5,
        'x': pointcloud.points[:, 0],
        'y': pointcloud.points[:, 1],
        'z': pointcloud.points[:, 2]
    }

    # 4. Extract all fields except excluded ones
    excluded_fields = {"vtkOriginalPointIds", "residuals"}

    for key, array in pointcloud.point_data.items():
        if key in excluded_fields:
            continue

        if array.ndim == 1:
            # Standard scalar field (e.g., intensity)
            data[key] = array
        elif array.ndim == 2:
            # Vector/multi-column field (e.g., RGB colors or normals)
            for col_idx in range(array.shape[1]):
                data[f"{key}_{col_idx}"] = array[:, col_idx]

    # 5. Export main CSV
    df = pd.DataFrame(data)
    csv_path = output_dir / f"{name}_{trial_num}.csv"
    df.to_csv(csv_path, index=False)
    print(f"[DataSaver] Saved CSV: {csv_path}")

    # 6. Save total point counts per scan to a separate CSV
    if "cloud_index" in df.columns:
        counts_df = (
            df.groupby("cloud_index", as_index=False)
            .size()
            .rename(columns={"size": "point_count"})
        )
        
        counts_csv_path = output_dir / f"point_counts_per_scan_{trial_num}.csv"
        counts_df.to_csv(counts_csv_path, index=False)
        print(f"[DataSaver] Saved Point Counts CSV: {counts_csv_path}")