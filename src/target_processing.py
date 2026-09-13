from src.rosbag_loader import ROSPointCloudLoader
import pyvista as pv
import numpy as np

def get_target_in_bounds(bag_path: str, topic_name: str, target_box_bounds: list):

    loader = ROSPointCloudLoader(bag_path)
    combined_scan = pv.PolyData()

    # get the combined pointcloud of all scans per bag
    combined_scan = loader.read_clouds(topic_name, True)
    target_subset = combined_scan.clip_box(bounds=target_box_bounds, invert=False)

    return target_subset

def fit_plane_irls(
    cloud: pv.PolyData,
    loss_function: str = "tukey",
    max_iters: int = 30,
    tol: float = 1e-6,
    inlier_threshold: float = 0.02,
    target_width: float = 0.4,
    target_height: float = 0.4,
):
    """Fits a plane using IRLS and crops points to a specified 2D lateral bounding box,

    removing vertical mounting posts below the target.
    """
    pts = cloud.points.copy()
    n_points = pts.shape[0]

    # --- Step 1: Longitudinal Filtering via IRLS Plane Fitting ---
    weights = np.ones(n_points)
    prev_normal = np.zeros(3)

    for i in range(max_iters):
        weight_sum = np.sum(weights)
        if weight_sum == 0:
            raise ValueError(
                "Weights collapsed to zero. Adjust scale or threshold."
            )

        centroid = np.sum(pts * weights[:, np.newaxis], axis=0) / weight_sum
        centered_pts = pts - centroid
        cov = centered_pts.T @ (weights[:, np.newaxis] * centered_pts)

        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        normal /= np.linalg.norm(normal)

        if i > 0 and np.abs(np.dot(normal, prev_normal)) > (1.0 - tol):
            break
        prev_normal = normal

        residuals = np.abs(centered_pts @ normal)
        mad = np.median(residuals)
        scale = max(1.4826 * mad, 1e-8)

        u_scale = residuals / scale

        if loss_function.lower() == "tukey":
            c = 4.685
            mask = u_scale <= c
            weights = np.zeros(n_points)
            weights[mask] = (1.0 - (u_scale[mask] / c) ** 2) ** 2
        elif loss_function.lower() == "huber":
            k = 1.345
            weights = np.ones(n_points)
            outliers = u_scale > k
            weights[outliers] = k / u_scale[outliers]
        elif loss_function.lower() == "cauchy":
            c = 2.385
            weights = 1.0 / (1.0 + (u_scale / c) ** 2)

    # Filter longitudinal (depth) inliers
    final_residuals = np.abs((pts - centroid) @ normal)
    planar_mask = final_residuals <= inlier_threshold

    # --- Step 2: Local 2D Basis Construction ---
    # Define vertical direction in local target coordinates (world Z projected onto plane)
    world_z = np.array([0.0, 0.0, 1.0])
    v_proj = world_z - np.dot(world_z, normal) * normal

    # Fallback if plane is completely horizontal
    if np.linalg.norm(v_proj) < 1e-3:
        world_y = np.array([0.0, 1.0, 0.0])
        v_proj = world_y - np.dot(world_y, normal) * normal

    v_local = v_proj / np.linalg.norm(v_proj)  # Local vertical axis (upwards)
    u_local = np.cross(v_local, normal)  # Local horizontal axis

    # --- Step 3: Lateral Projection & Bounding (Post Removal) ---
    pts_inliers = pts[planar_mask]
    pts_centered = pts_inliers - centroid

    u_coords = pts_centered @ u_local  # Horizontal offsets
    v_coords = pts_centered @ v_local  # Vertical offsets

    # Identify the top of the target (99th percentile to exclude stray top points)
    v_top = np.percentile(v_coords, 99)
    v_bottom = v_top - target_height

    # Horizontal center of the target face
    u_center = np.median(u_coords)
    u_left = u_center - (target_width / 2.0)
    u_right = u_center + (target_width / 2.0)

    # Lateral mask: keep points within the 0.4m x 0.4m window from top edge down
    lateral_mask = (
        (v_coords >= v_bottom)
        & (v_coords <= v_top)
        & (u_coords >= u_left)
        & (u_coords <= u_right)
    )

    # Combine indices back to original cloud
    inlier_indices = np.where(planar_mask)[0][lateral_mask]

    output_cloud = cloud.copy()
    output_cloud.point_data["residuals"] = final_residuals

    target_cloud = output_cloud.extract_points(
        inlier_indices, adjacent_cells=False
    )

    plane_params = {
        "centroid": np.mean(pts[inlier_indices], axis=0),
        "normal": normal,
        "rmse": np.sqrt(np.mean(final_residuals[inlier_indices] ** 2)),
    }

    return target_cloud, plane_params