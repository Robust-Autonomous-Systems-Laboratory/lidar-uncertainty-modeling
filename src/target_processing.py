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
):
    """Fits a plane to a PyVista PolyData point cloud using Iteratively Reweighted Least Squares (IRLS).

    Parameters
    ----------
    cloud : pv.PolyData
        Input PyVista point cloud.
    loss_function : str
        Robust loss weighting function: 'tukey' (hard rejection), 'huber', or
        'cauchy'.
    max_iters : int
        Maximum number of IRLS iterations.
    tol : float
        Convergence tolerance based on normal vector angular change.
    inlier_threshold : float
        Distance cutoff (in point cloud units) to extract the filtered inlier sub-cloud.

    Returns
    -------
    inlier_cloud : pv.PolyData
        Cleaned sub-cloud containing inliers with 'weights' and 'residuals' arrays attached.
    plane_params : dict
        Extracted plane attributes: 'centroid', 'normal', and 'rmse'.
    """
    pts = cloud.points.copy()
    n_points = pts.shape[0]

    # Initialize equal weights
    weights = np.ones(n_points)
    prev_normal = np.zeros(3)

    for i in range(max_iters):
        # 1. Weighted Centroid
        weight_sum = np.sum(weights)
        if weight_sum == 0:
            raise ValueError(
                "Weights collapsed to zero. Adjust scale or threshold."
            )
        centroid = np.sum(pts * weights[:, np.newaxis], axis=0) / weight_sum

        # 2. Weighted Covariance Matrix
        centered_pts = pts - centroid
        cov = centered_pts.T @ (weights[:, np.newaxis] * centered_pts)

        # 3. Normal Vector from Smallest Eigenvector
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        normal /= np.linalg.norm(normal)

        # Check convergence (cosine similarity of normal vector)
        if i > 0 and np.abs(np.dot(normal, prev_normal)) > (1.0 - tol):
            break
        prev_normal = normal

        # 4. Orthogonal Residuals
        residuals = np.abs(centered_pts @ normal)

        # 5. Robust Scale Estimation via Median Absolute Deviation (MAD)
        mad = np.median(residuals)
        scale = 1.4826 * mad
        scale = max(scale, 1e-8)  # Prevent division by zero

        # 6. Weight Adjustment
        u = residuals / scale

        if loss_function.lower() == "tukey":
            c = 4.685  # Standard Tukey tuning constant
            mask = u <= c
            weights = np.zeros(n_points)
            weights[mask] = (1.0 - (u[mask] / c) ** 2) ** 2

        elif loss_function.lower() == "huber":
            k = 1.345  # Standard Huber tuning constant
            weights = np.ones(n_points)
            outliers = u > k
            weights[outliers] = k / u[outliers]

        elif loss_function.lower() == "cauchy":
            c = 2.385  # Standard Cauchy tuning constant
            weights = 1.0 / (1.0 + (u / c) ** 2)

        else:
            raise ValueError(f"Unsupported loss function: {loss_function}")

    # Final calculations using converged parameters
    centered_pts = pts - centroid
    final_residuals = np.abs(centered_pts @ normal)

    # Attach computed scalar arrays to PolyData
    output_cloud = cloud.copy()
    output_cloud.point_data["irls_weights"] = weights
    output_cloud.point_data["residuals"] = final_residuals

    # Extract inliers using the spatial distance threshold
    inlier_mask = final_residuals <= inlier_threshold
    inlier_indices = np.where(inlier_mask)[0]
    inlier_cloud = output_cloud.extract_points(
        inlier_indices, adjacent_cells=False
    )

    plane_params = {
        "centroid": centroid,
        "normal": normal,
        "rmse": np.sqrt(np.mean(final_residuals[inlier_mask] ** 2)),
    }

    return inlier_cloud, plane_params