from src.rosbag_loader import ROSPointCloudLoader
import pyvista as pv
import numpy as np


def fmt_sig(value: float, sig: int = 4) -> str:
    """Fixed-point text with `sig` significant digits (trailing zeros kept, no exponent)."""
    if value is None or not np.isfinite(value):
        return "—"
    if value == 0:
        return f"{0:.{sig - 1}f}"
    decimals = max(0, sig - 1 - int(np.floor(np.log10(abs(value)))))
    text = f"{value:.{decimals}f}"
    # Rounding can carry into a new leading digit (9.9996 -> 10.000); drop the extra decimal
    if decimals > 0 and len(text.lstrip("-").replace(".", "").lstrip("0")) > sig:
        text = f"{value:.{decimals - 1}f}"
    return text


def fmt_cm(value_m: float) -> str:
    """A length in metres as centimetres to 4 significant digits (project reporting convention)."""
    return "—" if value_m is None or not np.isfinite(value_m) else fmt_sig(value_m * 100)

def get_target_in_bounds(bag_path: str, topic_name: str, target_box_bounds: list):

    loader = ROSPointCloudLoader(bag_path)
    combined_scan = pv.PolyData()

    # get the combined pointcloud of all scans per bag
    combined_scan = loader.read_clouds(topic_name, True)
    target_subset = combined_scan.clip_box(bounds=target_box_bounds, invert=False)

    return target_subset

def beam_range_residuals(pts: np.ndarray, centroid: np.ndarray, normal: np.ndarray,
                         min_cos_incidence: float = 1e-3) -> np.ndarray:
    """Signed range offset of each point from the plane, measured along its own beam.

    Assumes the sensor is at the cloud origin, so each point's beam runs from the origin
    through the point. For range r along unit direction d, the beam meets the plane at
    r_plane = n.c / n.d, so the residual is r - r_plane = n.(p - c) / n.d. Positive means
    the return is farther than the plane. Points at near-grazing incidence
    (|n.d| < min_cos_incidence) get NaN.
    """
    pts = np.asarray(pts, dtype=float)
    ranges = np.linalg.norm(pts, axis=1)
    directions = pts / ranges[:, np.newaxis]
    cos_incidence = directions @ normal
    plane_dist = (pts - centroid) @ normal

    residuals = np.full(len(pts), np.nan)
    ok = np.abs(cos_incidence) >= min_cos_incidence
    residuals[ok] = plane_dist[ok] / cos_incidence[ok]
    return residuals


# --- Fit method selection -------------------------------------------------------------
# A plane is only trusted when the target points spread in two directions on the target
# face. With one scan line across the target the points are collinear, the tilt about that
# line is unobservable, and the plane normal is arbitrary. Conditioning is judged from the
# covariance eigenvalues l0 <= l1 <= l2 of the final target points: sqrt(l1 / l2) is the
# ratio of the minor to major in-plane spread (1 = square patch, ~0 = a single line).
PLANE_MIN_POINTS = 10
PLANE_MIN_ASPECT = 0.2
# The plane must also face the sensor (at the cloud origin). One scan line plus range noise
# can look two-dimensional, but the "plane" through it then nearly contains the beams, and
# along-beam residuals (divided by the incidence cosine) are meaningless.
PLANE_MAX_INCIDENCE_DEG = 75.0

# Per-channel (ring / scan line) fallback: each channel needs this many points at this many
# distinct azimuths to fit the 3-parameter beam-range model.
RING_MIN_POINTS = 5
RING_MIN_AZIMUTHS = 3
RING_AZIMUTH_GAP = 2e-4  # rad; azimuths closer than this count as the same column

# Fixed-beam fallback for channels with too few azimuth columns for the line model. Far
# targets on a sparse lidar (e.g. Iris at >= 60 m) are often hit by a single column per scan
# line, but in every frame. A column whose azimuths stay within BEAM_MAX_AZIMUTH_SPAN is one
# fixed beam direction (a lidar with a frame-locked scan pattern); its range is fitted by a
# constant, so the residual is the beam's frame-to-frame range scatter. Columns of a lidar
# whose azimuths drift between frames span much more and are not used.
BEAM_MIN_POINTS = 5
BEAM_MAX_AZIMUTH_SPAN = 1e-3  # rad

# Point fields that identify a fixed-elevation channel, in order of preference
CHANNEL_FIELDS = ("ring", "line_index", "channel", "laser_id")

FIT_METHOD_PLANE = "plane"
FIT_METHOD_RING_LINE = "ring_line"
FIT_METHOD_BEAM = "beam"
FIT_METHOD_LABELS = {FIT_METHOD_PLANE: "Plane", FIT_METHOD_RING_LINE: "Ring line",
                     FIT_METHOD_BEAM: "Fixed beam"}


class TargetFitError(ValueError):
    """Neither a plane nor the per-ring line fallback could be fitted to the target."""


def channel_field(field_names) -> str | None:
    """First field in CHANNEL_FIELDS present in field_names, else None."""
    return next((name for name in CHANNEL_FIELDS if name in field_names), None)


def plane_conditioning(pts: np.ndarray) -> tuple[bool, str, float]:
    """(well conditioned, reason when not, aspect sqrt(l1/l2)) for a plane through pts."""
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    if n < 3:
        return False, f"only {n} target points", float("nan")
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(pts.T))
    aspect = float(np.sqrt(max(eigenvalues[1], 0.0) / eigenvalues[2])) if eigenvalues[2] > 0 else 0.0
    if n < PLANE_MIN_POINTS:
        return False, f"only {n} target points (plane needs {PLANE_MIN_POINTS})", aspect
    if aspect < PLANE_MIN_ASPECT:
        return False, (f"target points nearly collinear (in-plane aspect {aspect:.3f} "
                       f"< {PLANE_MIN_ASPECT})"), aspect
    centroid = pts.mean(axis=0)
    if np.linalg.norm(centroid) > 0:
        cos_incidence = abs(eigenvectors[:, 0] @ centroid) / np.linalg.norm(centroid)
        incidence = float(np.degrees(np.arccos(min(cos_incidence, 1.0))))
        if incidence > PLANE_MAX_INCIDENCE_DEG:
            return False, (f"plane through the points is edge-on to the sensor (incidence "
                           f"{incidence:.0f} deg > {PLANE_MAX_INCIDENCE_DEG:.0f})"), aspect
    return True, "", aspect


def _azimuth_columns(azimuth: np.ndarray) -> np.ndarray:
    """Column id per point: sorted azimuths further apart than RING_AZIMUTH_GAP start a new
    column."""
    if len(azimuth) == 0:
        return np.array([], dtype=int)
    order = np.argsort(azimuth)
    ids = np.empty(len(azimuth), dtype=int)
    ids[order] = np.concatenate([[0], np.cumsum(np.diff(azimuth[order]) > RING_AZIMUTH_GAP)])
    return ids


def _count_azimuth_columns(azimuth: np.ndarray) -> int:
    return int(_azimuth_columns(azimuth).max() + 1) if len(azimuth) else 0


def _fixed_beam_residuals(ranges: np.ndarray, azimuth: np.ndarray) -> tuple[np.ndarray, int]:
    """Residuals about each fixed beam's mean range, for one channel's points.

    :return: (residual per point, NaN outside usable beams; number of beams fitted)
    """
    residuals = np.full(len(ranges), np.nan)
    columns = _azimuth_columns(azimuth)
    n_beams = 0
    for column in np.unique(columns):
        idx = np.where(columns == column)[0]
        n = len(idx)
        if n < BEAM_MIN_POINTS or np.ptp(azimuth[idx]) > BEAM_MAX_AZIMUTH_SPAN:
            continue
        # One fitted parameter (the beam's mean range): rescale by sqrt(n / (n - 1))
        residuals[idx] = (ranges[idx] - ranges[idx].mean()) * np.sqrt(n / (n - 1))
        n_beams += 1
    return residuals, n_beams


def ring_line_range_residuals(pts: np.ndarray, channels: np.ndarray) -> tuple[np.ndarray, int, int]:
    """Range residuals from a separate fit of each channel's points against azimuth.

    Every point of one channel shares an elevation angle phi, so for a planar target its
    range r at azimuth a satisfies exactly 1/r = A cos(a) + B sin(a) + C. That is linear in
    (A, B, C) and is fitted by least squares per channel, pooling all scans. Tilt about the
    horizontal axis only shifts a channel's ranges by a constant, which C absorbs, so the
    unobservable tilt of a one-line target does not enter the residuals. Each channel also
    gets its own range offset, so systematic offsets between channels are excluded.

    Residuals are scaled per channel by sqrt(n / (n - 3)) to correct for the 3 fitted
    parameters, so their spread estimates range noise without a degrees-of-freedom bias.

    A channel with too few azimuth columns for the line model falls back to fixed-beam fits
    (see _fixed_beam_residuals): each repeatedly sampled beam direction gets its own constant
    range, so its residuals are pure frame-to-frame range scatter.

    :return: (signed residual r - r_fit per point, positive = farther; NaN for points in
              channels that could not be fitted, number of channels line-fitted, number of
              fixed beams fitted in the remaining channels)
    """
    pts = np.asarray(pts, dtype=float)
    ranges = np.linalg.norm(pts, axis=1)
    azimuth = np.unwrap(np.arctan2(pts[:, 1], pts[:, 0]))
    residuals = np.full(len(pts), np.nan)
    n_fitted = n_beams = 0
    for channel in np.unique(channels):
        idx = np.where(channels == channel)[0]
        if len(idx) < RING_MIN_POINTS or _count_azimuth_columns(azimuth[idx]) < RING_MIN_AZIMUTHS:
            residuals[idx], n = _fixed_beam_residuals(ranges[idx], azimuth[idx])
            n_beams += n
            continue
        a = azimuth[idx] - azimuth[idx].mean()  # centred azimuth keeps the system well scaled
        # Same span as [cos a, sin a, 1], rearranged so the columns are not near-collinear
        design = np.column_stack([np.ones_like(a), np.sin(a), 1.0 - np.cos(a)])
        inv_r = 1.0 / ranges[idx]
        coeffs, *_ = np.linalg.lstsq(design, inv_r, rcond=None)
        inv_fit = design @ coeffs
        if np.any(inv_fit <= 0):
            continue
        # Each channel spends 3 degrees of freedom on (A, B, C), so its raw residuals
        # understate the noise by sqrt((n - 3) / n). Rescale so the pooled residuals have
        # E[sum r^2] = N sigma^2 and a plain std over them is unbiased for range noise.
        n = len(idx)
        residuals[idx] = (ranges[idx] - 1.0 / inv_fit) * np.sqrt(n / (n - 3))
        n_fitted += 1
    return residuals, n_fitted, n_beams


def target_range_residuals(pts: np.ndarray, channels: np.ndarray | None = None,
                           plane: tuple[np.ndarray, np.ndarray] | None = None) -> tuple[np.ndarray, str, str]:
    """Along-beam range residuals for target points, choosing the fit method.

    Fallback order: plane (if well conditioned) > per-ring line, with fixed-beam fits for
    channels that have too few azimuth columns > TargetFitError. The method is "ring_line"
    when any channel was line-fitted, else "beam".

    :param channels: per-point channel id (ring / line index), or None if the lidar has none
    :param plane: (centroid, normal) to use when the plane is well conditioned; a
        least-squares plane through pts is fitted when omitted
    Both methods scale residuals by sqrt(n / (n - 3)) (per fit) to correct for the 3 fitted
    parameters, so the residual spread is an unbiased estimate of range noise.

    :return: (residuals, NaN where unusable; fit method; human-readable detail)
    """
    pts = np.asarray(pts, dtype=float)
    ok, reason, aspect = plane_conditioning(pts)
    if ok:
        if plane is None:
            centroid = pts.mean(axis=0)
            _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
            plane = (centroid, vh[-1])
        # The plane (fitted to these same points) spends 3 degrees of freedom; rescale as
        # in ring_line_range_residuals so the residual spread is unbiased for range noise.
        n = len(pts)
        residuals = beam_range_residuals(pts, *plane) * np.sqrt(n / (n - 3))
        return residuals, FIT_METHOD_PLANE, f"aspect {aspect:.2f}"

    if channels is None:
        raise TargetFitError(f"plane fit rejected: {reason}; ring-line fallback unavailable "
                             f"(no {'/'.join(CHANNEL_FIELDS)} field)")
    residuals, n_fitted, n_beams = ring_line_range_residuals(pts, np.asarray(channels))
    n_valid = int(np.sum(~np.isnan(residuals)))
    if (n_fitted == 0 and n_beams == 0) or n_valid < RING_MIN_POINTS:
        raise TargetFitError(
            f"plane fit rejected: {reason}; ring-line fallback failed (no channel had "
            f">= {RING_MIN_POINTS} points at >= {RING_MIN_AZIMUTHS} distinct azimuths, and no "
            f"fixed beam had >= {BEAM_MIN_POINTS} hits)")
    parts = ([f"{n_fitted} channel(s)"] if n_fitted else []) + ([f"{n_beams} fixed beam(s)"] if n_beams else [])
    method = FIT_METHOD_RING_LINE if n_fitted else FIT_METHOD_BEAM
    return residuals, method, f"{' + '.join(parts)}; plane rejected: {reason}"


# --- Target face segmentation ----------------------------------------------------------
# Candidate face points are those within FACE_SEARCH_BAND of the plane. The face is then
# located as the full-size target rectangle (in plane coordinates) covering the most
# occupied area, so stray points, mounting posts and the top edge do not position the window.
# Points are kept inside that rectangle (less EDGE_MARGIN, which drops edge mixed pixels)
# and within a depth gate of DEPTH_GATE_K robust sigmas of the plane: a gate sized to the
# measured noise keeps all target returns, where a fixed gate tighter than the noise would
# both drop points and truncate (bias low) the repeatability.
FACE_SEARCH_BAND = 0.25     # m; loose plane-distance band for locating the face
FACE_GRID_STEP = 0.01       # m; grid resolution of the rectangle search
EDGE_MARGIN = 0.02          # m; trimmed from each side of the target rectangle
DEPTH_GATE_K = 3.0          # depth gate in robust sigmas (1.4826 * MAD)
DEPTH_GATE_MIN = 0.005      # m; lower bound on the gate for very low-noise sensors
# A scan line (channel) crossing the target face spans its full width; one crossing only a
# mounting post or a stray edge is narrow. Lines narrower than this fraction of the widest
# line in the window are dropped.
SCAN_LINE_MIN_COVER = 0.5
# A scan line straddling the top or bottom edge of the target (or clipping its post) puts
# only part of its beam footprint on the face: weak, mixed returns with several times the
# range noise of lines fully on the face. Lines whose robust plane-distance sigma exceeds
# this multiple of the typical line's (median of >= 3 lines, else the quieter line) are
# dropped as edge lines. Lines need SCAN_LINE_MIN_POINTS points to be judged.
SCAN_LINE_MAX_NOISE_RATIO = 2.0
SCAN_LINE_MIN_POINTS = 10
# Without a channel field, scan lines are recovered by clustering beam elevation angles;
# elevations further apart than this start a new line (well below typical channel spacing).
SCAN_LINE_ELEVATION_GAP = np.radians(0.05)


def _plane_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(u, v) in-plane axes: v is world Z projected onto the plane (world Y for a horizontal
    plane), u completes the right-handed frame (horizontal across the target face)."""
    world_z = np.array([0.0, 0.0, 1.0])
    v_proj = world_z - np.dot(world_z, normal) * normal
    if np.linalg.norm(v_proj) < 1e-3:
        world_y = np.array([0.0, 1.0, 0.0])
        v_proj = world_y - np.dot(world_y, normal) * normal
    v_local = v_proj / np.linalg.norm(v_proj)
    return np.cross(v_local, normal), v_local


def _locate_face(u: np.ndarray, v: np.ndarray, width: float, height: float,
                 step: float = FACE_GRID_STEP) -> tuple[float, float]:
    """Centre (u, v) of the width x height rectangle covering the most occupied grid cells.

    Occupied area rather than point count is maximised, so a densely sampled row (e.g. edge
    bloom where the target meets its post) cannot outweigh face rows with fewer returns.
    Uses a summed-area table over a step-sized grid. When several positions tie (the points
    are smaller than the rectangle in some direction), the middle one is taken so the
    rectangle is centred on the points.
    """
    wu, wv = max(1, int(round(width / step))), max(1, int(round(height / step)))
    # Pad by one window on each side so the rectangle can also overhang the outermost points
    u0, v0 = u.min() - wu * step, v.min() - wv * step
    nu = int(np.ceil((u.max() - u0) / step)) + 1 + wu
    nv = int(np.ceil((v.max() - v0) / step)) + 1 + wv
    grid, _, _ = np.histogram2d(u, v, bins=[nu, nv],
                                range=[[u0, u0 + nu * step], [v0, v0 + nv * step]])
    sat = np.zeros((nu + 1, nv + 1))
    sat[1:, 1:] = (grid > 0).cumsum(axis=0).cumsum(axis=1)
    counts = sat[wu:, wv:] - sat[:-wu, wv:] - sat[wu:, :-wv] + sat[:-wu, :-wv]
    iu, iv = np.nonzero(counts == counts.max())
    return u0 + (np.median(iu) + wu / 2) * step, v0 + (np.median(iv) + wv / 2) * step


def _robust_sigma(values: np.ndarray) -> float:
    return float(1.4826 * np.median(np.abs(values - np.median(values)))) if len(values) else 0.0


def elevation_scan_lines(pts: np.ndarray, gap: float = SCAN_LINE_ELEVATION_GAP) -> np.ndarray:
    """Scan-line id per point from its beam elevation angle, for lidars without a channel
    field. Each laser has a fixed elevation, so sorted elevations form tight clusters
    separated by the channel spacing; a jump larger than gap (rad) starts a new line."""
    pts = np.asarray(pts, dtype=float)
    elevation = np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1]))
    order = np.argsort(elevation)
    ids = np.empty(len(pts), dtype=int)
    ids[order] = np.concatenate([[0], np.cumsum(np.diff(elevation[order]) > gap)]) if len(pts) else []
    return ids


def _partial_scan_lines(lines: np.ndarray, u: np.ndarray, signed: np.ndarray,
                        in_rect: np.ndarray, keep: np.ndarray) -> tuple[list, list]:
    """(narrow lines, noisy lines) among the scan lines in the target rectangle.

    Narrow: kept points span less than SCAN_LINE_MIN_COVER of the widest line (post only).
    Noisy: robust plane-distance sigma of the line's points in the rectangle above
    SCAN_LINE_MAX_NOISE_RATIO times the typical line's (edge-straddling beams).
    """
    if not np.any(keep):
        return [], []
    widths = {line: np.ptp(u[keep & (lines == line)]) for line in np.unique(lines[keep])}
    widest = max(widths.values())
    narrow = [line for line, w in widths.items() if w < SCAN_LINE_MIN_COVER * widest]

    sigmas = {}
    for line in widths:
        if line in narrow:
            continue
        d = signed[in_rect & (lines == line)]
        if len(d) >= SCAN_LINE_MIN_POINTS:
            sigmas[line] = _robust_sigma(d)
    noisy = []
    if len(sigmas) >= 2:
        values = np.array(list(sigmas.values()))
        typical = np.median(values) if len(values) >= 3 else values.min()
        noisy = [line for line, sg in sigmas.items() if sg > SCAN_LINE_MAX_NOISE_RATIO * typical]
    return narrow, noisy


def _segment_target(pts, centroid, normal, target_width, target_height, lines=None,
                    inlier_threshold=None, edge_margin=EDGE_MARGIN, max_passes=3,
                    exclude_lines=()):
    """Indices of the points on the target face.

    :param lines: per-point scan-line id (channel field or elevation_scan_lines), or None.
        Lines that only cross a mounting post (narrow) or straddle a target edge (noisy)
        are dropped and the face is located again without them, so they cannot pull the
        target rectangle off the face or inflate the depth gate.
    :param exclude_lines: scan lines already identified as post / edge lines
    :param inlier_threshold: fixed depth gate (m); None sizes it from the measured noise
    :return: (indices into pts, signed plane distance of every point, info dict)
    """
    signed = (pts - centroid) @ normal
    info = {"gate": float("nan"), "sigma": float("nan"), "dropped_lines": 0, "saturated": False,
            "dropped_ids": [], "dropped_narrow": 0, "dropped_noisy": 0}
    candidates = np.abs(signed) <= FACE_SEARCH_BAND
    dropped = list(exclude_lines) if lines is not None else []
    if dropped:
        candidates &= ~np.isin(lines, dropped)
    if not np.any(candidates):
        return np.array([], dtype=int), signed, info

    u_axis, v_axis = _plane_axes(normal)
    rel = pts - centroid
    u, v = rel @ u_axis, rel @ v_axis
    for _ in range(max_passes):
        u_c, v_c = _locate_face(u[candidates], v[candidates], target_width, target_height)
        in_rect = (candidates
                   & (np.abs(u - u_c) <= target_width / 2.0 - edge_margin)
                   & (np.abs(v - v_c) <= target_height / 2.0 - edge_margin))

        if inlier_threshold is None:
            sigma = _robust_sigma(signed[in_rect])
            gate = min(max(DEPTH_GATE_K * sigma, DEPTH_GATE_MIN), FACE_SEARCH_BAND)
            info["sigma"] = sigma
            info["saturated"] = DEPTH_GATE_K * sigma >= FACE_SEARCH_BAND
        else:
            gate = inlier_threshold
        keep = in_rect & (np.abs(signed) <= gate)

        narrow, noisy = (_partial_scan_lines(lines, u, signed, in_rect, keep)
                         if lines is not None else ([], []))
        if not narrow and not noisy:
            break
        dropped += narrow + noisy
        info["dropped_narrow"] += len(narrow)
        info["dropped_noisy"] += len(noisy)
        candidates &= ~np.isin(lines, narrow + noisy)
        if not np.any(candidates):
            keep = candidates
            break
    else:
        keep &= ~np.isin(lines, dropped)

    info["gate"] = gate
    info["dropped_ids"] = dropped
    info["dropped_lines"] = len(dropped)
    return np.where(keep)[0], signed, info


# --- Reflectivity separation of adjacent targets ---------------------------------------
# Targets placed side by side at long range are nearly coplanar and a search box (plus the
# per-lidar frame offset) can hold returns from both. When the experiment config gives an
# adjacent target a clearly different nominal reflectivity, the box points are split into
# one class per reflectivity level by their measured return strength before any plane
# fitting, and only the class whose rank matches this target's reflectivity is kept.
# Calibrated fields are preferred; raw intensity still ranks targets within one box.
REFLECTIVITY_FIELDS = ("reflectance", "reflectivity", "intensity")
ADJACENT_TARGET_MARGIN = 0.5      # m; search boxes grown by this much that overlap are adjacent
REFLECTIVITY_LEVEL_RATIO = 2.0    # nominal reflectivities this far apart are separable levels
SPLIT_MIN_CONTRAST = 1.5          # measured median return ratio across a class boundary
SPLIT_MIN_SEPARABILITY = 0.5      # Otsu between-class / total variance across a boundary
SPLIT_MIN_FRACTION = 0.02         # smallest class, as a fraction of the box points
SPLIT_HIST_BINS = 128


def reflectivity_field(field_names) -> str | None:
    """First field in REFLECTIVITY_FIELDS present in field_names, else None."""
    return next((name for name in REFLECTIVITY_FIELDS if name in field_names), None)


def adjacent_target_reflectivities(targets: dict, name) -> dict:
    """{other target: nominal reflectivity} for targets whose search boxes, each grown by
    ADJACENT_TARGET_MARGIN, overlap the search box of target `name` (experiment config)."""
    cur = targets[name]
    centre = np.array([cur["x_target"], cur["y_target"], cur["z_target"]], dtype=float)
    extent = np.array([cur["x_extent"], cur["y_extent"], cur["z_extent"]], dtype=float)
    neighbours = {}
    for other, cfg in targets.items():
        if other == name or cfg.get("reflectivity") is None:
            continue
        o_centre = np.array([cfg["x_target"], cfg["y_target"], cfg["z_target"]], dtype=float)
        o_extent = np.array([cfg["x_extent"], cfg["y_extent"], cfg["z_extent"]], dtype=float)
        if np.all(np.abs(centre - o_centre) <= (extent + o_extent) / 2.0 + 2 * ADJACENT_TARGET_MARGIN):
            neighbours[other] = float(cfg["reflectivity"])
    return neighbours


def _reflectivity_levels(values) -> list[list[float]]:
    """Group nominal reflectivities into levels: sorted ascending, a new level starts when a
    value is at least REFLECTIVITY_LEVEL_RATIO times the first value of the current level."""
    levels = []
    for value in sorted(values):
        if levels and value < REFLECTIVITY_LEVEL_RATIO * levels[-1][0]:
            levels[-1].append(value)
        else:
            levels.append([value])
    return levels


def _multilevel_otsu(values: np.ndarray, n_classes: int, bins: int = SPLIT_HIST_BINS) -> np.ndarray:
    """Thresholds (ascending, n_classes - 1) maximising between-class variance of values,
    by dynamic programming over a histogram."""
    counts, edges = np.histogram(values, bins=bins)
    centres = (edges[:-1] + edges[1:]) / 2.0
    P = np.concatenate([[0.0], np.cumsum(counts)])
    S = np.concatenate([[0.0], np.cumsum(counts * centres)])

    def score(i, j):  # class holding bins i..j-1 (vectorised over i)
        n = P[j] - P[i]
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(n > 0, (S[j] - S[i]) ** 2 / np.maximum(n, 1e-12), 0.0)

    best = np.full((n_classes + 1, bins + 1), -np.inf)
    arg = np.zeros((n_classes + 1, bins + 1), dtype=int)
    best[0, 0] = 0.0
    for c in range(1, n_classes + 1):
        for j in range(c, bins + 1):
            i = np.arange(c - 1, j)
            cand = best[c - 1, i] + score(i, j)
            k = int(np.argmax(cand))
            best[c, j], arg[c, j] = cand[k], i[k]
    cuts, j = [], bins
    for c in range(n_classes, 1, -1):
        j = arg[c, j]
        cuts.append(edges[j])
    return np.array(sorted(cuts))


def _boundary_ok(low: np.ndarray, high: np.ndarray, n_total: int) -> bool:
    """Whether two neighbouring classes (log values) are distinct populations."""
    if min(len(low), len(high)) < max(3, SPLIT_MIN_FRACTION * n_total):
        return False
    if np.median(high) - np.median(low) < np.log(SPLIT_MIN_CONTRAST):
        return False
    both = np.concatenate([low, high])
    total_var = np.var(both)
    if total_var <= 0:
        return False
    w0, w1 = len(low) / len(both), len(high) / len(both)
    return w0 * w1 * (low.mean() - high.mean()) ** 2 / total_var >= SPLIT_MIN_SEPARABILITY


def separate_by_reflectivity(cloud: pv.PolyData, reflectivity, neighbours: dict):
    """Keep the points of the reflectivity class that matches this target.

    :param reflectivity: this target's nominal reflectivity (None skips the split)
    :param neighbours: {target: nominal reflectivity} of adjacent targets
        (see adjacent_target_reflectivities)
    :return: (cloud with only this target's class, or the input cloud unchanged; note
        describing what was done, "" when no split was attempted)
    """
    if reflectivity is None or cloud.n_points == 0:
        return cloud, ""
    reflectivity = float(reflectivity)
    levels = _reflectivity_levels([reflectivity, *neighbours.values()])
    if len(levels) < 2:
        return cloud, ""  # every adjacent target has a similar reflectivity
    other_names = [n for n, r in neighbours.items()
                   if not any(reflectivity in lv and r in lv for lv in levels)]
    field = reflectivity_field(cloud.point_data.keys())
    if field is None:
        return cloud, f"no reflectivity split from {', '.join(other_names)} (no return-strength field)"

    values = np.asarray(cloud.point_data[field], dtype=float)
    positive = values[values > 0]
    if len(positive) == 0:
        return cloud, f"no reflectivity split from {', '.join(other_names)} (no positive {field})"
    log_v = np.log(np.maximum(values, positive.min() / 2.0))
    own_rank = next(i for i, lv in enumerate(levels) if reflectivity in lv)

    # Fit one class per level; drop boundaries that are not distinct populations (e.g. a
    # neighbour that is not in this box) and refit with fewer classes.
    for n_classes in range(len(levels), 1, -1):
        cuts = _multilevel_otsu(log_v, n_classes)
        labels = np.searchsorted(cuts, log_v, side="right")
        if all(_boundary_ok(log_v[labels == c], log_v[labels == c + 1], len(log_v))
               for c in range(n_classes - 1)):
            break
    else:
        return cloud, (f"no reflectivity split from {', '.join(other_names)} "
                       f"({field} not bimodal: neighbour not in the search box)")

    if n_classes == len(levels):
        keep_class = own_rank
    elif own_rank == 0:
        keep_class = 0
    elif own_rank == len(levels) - 1:
        keep_class = n_classes - 1
    else:
        return cloud, (f"no reflectivity split from {', '.join(other_names)} "
                       f"(only {n_classes} of {len(levels)} {field} levels found)")

    keep = labels == keep_class
    side = ("brightest" if keep_class == n_classes - 1 else
            "dimmest" if keep_class == 0 else f"class {keep_class + 1}/{n_classes}")
    note = (f"reflectivity split from {', '.join(other_names)}: kept {int(keep.sum())}/"
            f"{len(keep)} {side} pts by {field}")
    return cloud.extract_points(np.where(keep)[0], adjacent_cells=False), note


def _line_of_sight_normal(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(centroid, normal) for collinear points: the plane containing their line that faces
    the sensor (normal = line of sight with its component along the line removed)."""
    centroid = pts.mean(axis=0)
    line_of_sight = centroid / np.linalg.norm(centroid)
    if len(pts) >= 2:
        _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
        direction = vh[0]
        normal = line_of_sight - np.dot(line_of_sight, direction) * direction
        if np.linalg.norm(normal) > 1e-6:
            return centroid, normal / np.linalg.norm(normal)
    return centroid, line_of_sight


# --- Clutter scan-line retry -------------------------------------------------------------
# A scan line through vegetation or ground in the search box returns ranges spread over tens
# of centimetres and can outnumber the few lines on a far target, so the IRLS plane locks
# onto the clutter and the face scatter fills the search band. When the fit fails, scan
# lines whose own robust range sigma would alone saturate the depth gate (DEPTH_GATE_K *
# sigma >= FACE_SEARCH_BAND) are dropped and the fit is retried on the remaining lines.
# Lines on a target face facing the sensor vary in range by only a few centimetres.
CLUTTER_LINE_MIN_SIGMA = FACE_SEARCH_BAND / DEPTH_GATE_K


def _scan_lines(cloud: pv.PolyData, pts: np.ndarray) -> np.ndarray:
    """Per-point scan-line id: the channel field, else clustered beam elevations."""
    field = channel_field(cloud.point_data.keys())
    return np.asarray(cloud.point_data[field]) if field else elevation_scan_lines(pts)


def depth_incoherent_scan_lines(cloud: pv.PolyData) -> list:
    """Scan lines (with >= SCAN_LINE_MIN_POINTS points) whose robust range sigma is at least
    CLUTTER_LINE_MIN_SIGMA. Empty when every line is incoherent (nothing would remain)."""
    pts = cloud.points.astype(float)
    if len(pts) == 0:
        return []
    lines = _scan_lines(cloud, pts)
    ranges = np.linalg.norm(pts, axis=1)
    clutter, kept = [], 0
    for line in np.unique(lines):
        r = ranges[lines == line]
        if len(r) >= SCAN_LINE_MIN_POINTS and _robust_sigma(r) >= CLUTTER_LINE_MIN_SIGMA:
            clutter.append(line)
        else:
            kept += len(r)
    return clutter if kept >= PLANE_MIN_POINTS else []


def fit_plane_irls(cloud: pv.PolyData, **kwargs):
    """Fits the target (see _fit_plane_irls_once). If that fails, drops depth-incoherent
    clutter scan lines from the search box (see depth_incoherent_scan_lines) and retries;
    the original error is raised when there are none or the retry also fails."""
    try:
        return _fit_plane_irls_once(cloud, **kwargs)
    except TargetFitError as first_error:
        clutter = depth_incoherent_scan_lines(cloud)
        if not clutter:
            raise
        lines = _scan_lines(cloud, cloud.points)
        keep = np.where(~np.isin(lines, clutter))[0]
        try:
            target_cloud, plane_params = _fit_plane_irls_once(
                cloud.extract_points(keep, adjacent_cells=False), **kwargs)
        except TargetFitError as retry_error:
            raise TargetFitError(f"{first_error}; retry without {len(clutter)} clutter scan "
                                 f"line(s) also failed: {retry_error}") from first_error
        plane_params["fit_detail"] += (f", excluded {len(clutter)} depth-incoherent clutter "
                                       f"scan line(s) after first fit failed")
        plane_params["clutter_scan_lines"] = len(clutter)
        return target_cloud, plane_params


def _fit_plane_irls_once(
    cloud: pv.PolyData,
    loss_function: str = "tukey",
    max_iters: int = 30,
    tol: float = 1e-6,
    inlier_threshold: float | None = None,
    target_width: float = 0.5,
    target_height: float = 0.5,
    edge_margin: float = EDGE_MARGIN,
    refine_iters: int = 2,
):
    """Fits a plane using IRLS, segments the target face, and computes range residuals.

    The IRLS plane over the search box seeds the segmentation (see _segment_target); the
    plane is then refitted to the face points alone and the face re-segmented, so clutter
    in the box does not tilt the final plane. Range residuals use the plane when the face
    points are well conditioned, otherwise a per-ring line fit (see target_range_residuals).
    Raises TargetFitError when neither can be fitted.

    :param inlier_threshold: fixed depth gate (m); None (default) sizes it to
        DEPTH_GATE_K robust sigmas of the measured plane-distance noise
    :param target_width, target_height: physical target size (m)
    :param edge_margin: trimmed from each side of the target rectangle (m)
    """
    pts = cloud.points.astype(float)
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

    # --- Steps 2-3: segment the target face, refitting the plane to the face points ---
    # Signed orthogonal distance to the plane removes geometric range variation across the
    # target face (e.g. a target not square-on to the lidar), leaving measurement scatter.
    field = channel_field(cloud.point_data.keys())
    all_channels = np.asarray(cloud.point_data[field]) if field else None

    # Scan lines for post rejection: the channel field, else clustered beam elevations
    scan_lines = _scan_lines(cloud, pts)

    def segment(c, n, exclude=()):
        return _segment_target(pts, c, n, target_width, target_height, scan_lines,
                               inlier_threshold, edge_margin, exclude_lines=exclude)

    inlier_indices, signed_residuals, seg_info = segment(centroid, normal)
    for _ in range(refine_iters):
        face = pts[inlier_indices]
        if not plane_conditioning(face)[0]:
            break
        centroid = face.mean(axis=0)
        normal = np.linalg.svd(face - centroid, full_matrices=False)[2][-1]
        inlier_indices, signed_residuals, seg_info = segment(centroid, normal)

    # --- Step 4: fit method selection ---
    # If the face points are collinear (one scan line), the IRLS normal is arbitrary and
    # the crop above may be wrong. Re-segment with the plane through that line that faces
    # the sensor, then let target_range_residuals fall back to the per-ring line fit.
    if not plane_conditioning(pts[inlier_indices])[0]:
        seed = pts[inlier_indices] if len(inlier_indices) >= 2 else pts
        centroid, normal = _line_of_sight_normal(seed)
        # Keep the post / edge lines already found, so they cannot return in the re-segment
        inlier_indices, signed_residuals, seg_info = segment(centroid, normal,
                                                             seg_info["dropped_ids"])

    if len(inlier_indices) == 0:
        raise TargetFitError("no points left after plane fitting")
    if seg_info["saturated"]:
        # Scatter as wide as the search band: no coherent surface (e.g. a few stray returns)
        raise TargetFitError(
            f"no coherent target surface: depth scatter sigma {fmt_cm(seg_info['sigma'])} cm "
            f"fills the +/-{fmt_cm(FACE_SEARCH_BAND)} cm search band")

    channels = all_channels[inlier_indices] if field else None
    # plane=None: least-squares plane through exactly these points, matching the
    # degrees-of-freedom correction applied to the residuals
    range_residual, method, detail = target_range_residuals(pts[inlier_indices], channels)

    # Keep only points with a usable residual (drops channels the ring-line fit skipped)
    usable = ~np.isnan(range_residual)
    inlier_indices, range_residual = inlier_indices[usable], range_residual[usable]
    detail += f"; {len(inlier_indices)} face pts, depth gate \u00b1{fmt_cm(seg_info['gate'])} cm"
    if seg_info["dropped_lines"]:
        detail += f", dropped {seg_info['dropped_lines']} post/edge scan line(s)"
    final_residuals = np.abs(signed_residuals)

    output_cloud = cloud.copy()
    output_cloud.point_data["residuals"] = final_residuals
    output_cloud.point_data["plane_distance"] = signed_residuals
    # Range offset along each point's beam from the fitted plane or channel line (used for s_r)
    full_range_residual = np.full(n_points, np.nan)
    full_range_residual[inlier_indices] = range_residual
    output_cloud.point_data["range_residual"] = full_range_residual
    target_cloud = output_cloud.extract_points(inlier_indices, adjacent_cells=False)

    plane_params = {
        "centroid": np.mean(pts[inlier_indices], axis=0),
        "normal": normal,
        "rmse": np.sqrt(np.mean(final_residuals[inlier_indices] ** 2)),
        "fit_method": method,
        "fit_detail": detail,
        "channel_field": field,
        "depth_gate": seg_info["gate"],
        "noise_sigma": seg_info["sigma"],
        "dropped_scan_lines": seg_info["dropped_lines"],
        "clutter_scan_lines": 0,
    }

    return target_cloud, plane_params
