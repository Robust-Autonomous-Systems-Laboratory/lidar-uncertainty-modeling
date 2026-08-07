from pathlib import Path
import pyvista as pv
import pandas as pd

def save_data(pointcloud: pv.DataSet, output_dir: Path | str, name: str = "control_volume_points")-> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Save as VTK file
    vtk_path = output_dir / f"{name}.vtk"
    pointcloud.save(str(vtk_path))

    # 2. Extract XYZ coordinates
    data = {
        'x': pointcloud.points[:, 0],
        'y': pointcloud.points[:, 1],
        'z': pointcloud.points[:, 2]
    }

    # 3. Extract all fields from point_data (e.g., intensity, ring, timestamp, etc.)
    for key, array in pointcloud.point_data.items():
        if array.ndim == 1:
            # Standard scalar field (e.g., intensity)
            data[key] = array
        elif array.ndim == 2:
            # Vector/multi-column field (e.g., RGB colors or normals)
            for col_idx in range(array.shape[1]):
                data[f"{key}_{col_idx}"] = array[:, col_idx]

    # 4. Export to CSV
    df = pd.DataFrame(data)
    csv_path = output_dir / f"{name}.csv"
    df.to_csv(csv_path, index=False)

    print(f"[DataSaver] Saved VTK: {vtk_path}")
    print(f"[DataSaver] Saved CSV: {csv_path}")