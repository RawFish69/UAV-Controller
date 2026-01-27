"""Mountains terrain generator.

This generator produces a *connected* mountain surface by sampling a heightfield
constructed from a mixture of peaks (gaussians), ridges, and pits/valleys.

The surface is returned as a `HeightFieldTerrain` so that:
- planners can route in XY without mountains "blocking everything"
- collision checks can enforce flying above ground (z > terrain(x,y))
"""

import numpy as np

from .obstacles import CylinderObstacle, HeightFieldTerrain


def generate_mountains(space_dim, num_peaks=15, base_size_range=(3.0, 8.0),
                       height_range=(10.0, 30.0), start_pos=None,
                       min_start_distance=5.0,
                       grid_resolution=10.0,
                       ridge_count=2,
                       ridge_chain_count=2,
                       ridge_chain_points=4,
                       ridge_chain_width=18.0,
                       ridge_chain_peak_spacing=18.0,
                       ridge_chain_peak_boost=1.0,
                       pit_count=6,
                       pit_depth_range=(5.0, 20.0),
                       steepness=1.6,
                       min_height=0.5):
    """
    Generate a mountainous terrain with a connected surface + optional rocks.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        num_peaks: Number of gaussian peaks contributing to the heightfield
        base_size_range: (min, max) "base size" for peaks (mapped to gaussian sigma)
        height_range: (min, max) peak amplitude range
        start_pos: Start position to avoid
        min_start_distance: Minimum distance from start
        grid_resolution: Cell size [m] for the heightfield-to-box conversion
        ridge_count: Number of ridge features to add (infinite lines, legacy-ish)
        ridge_chain_count: Number of *finite* ridge chains (polyline mountains)
        ridge_chain_points: Number of control points per ridge chain polyline
        ridge_chain_width: Approx ridge half-width [m] (gaussian sigma)
        ridge_chain_peak_spacing: Peak spacing along each ridge chain [m]
        ridge_chain_peak_boost: Multiplier on peak amplitudes along the chain
        pit_count: Number of pits/valleys ("hollow gaps") to subtract
        pit_depth_range: (min, max) depth range for pits
        steepness: >1.0 makes slopes more aggressive (nonlinear contrast)
        min_height: heights below this are ignored (reduces obstacle count)
        
    Returns:
        list: [HeightFieldTerrain, ...rocks...]
    """
    obstacles: list[HeightFieldTerrain | CylinderObstacle] = []
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)

    # ---------------------------------------------------------------------
    # Heightfield grid (cell centers)
    # ---------------------------------------------------------------------
    dx = float(grid_resolution)
    dy = float(grid_resolution)
    dx = max(0.5, dx)
    dy = max(0.5, dy)

    xs = np.arange(dx / 2.0, float(space_dim[0]) - dx / 2.0 + 1e-6, dx, dtype=float)
    ys = np.arange(dy / 2.0, float(space_dim[1]) - dy / 2.0 + 1e-6, dy, dtype=float)

    if xs.size == 0 or ys.size == 0:
        return obstacles

    X, Y = np.meshgrid(xs, ys, indexing="ij")
    H = np.zeros_like(X, dtype=float)

    def _distance_to_polyline(px: np.ndarray, py: np.ndarray, pts: np.ndarray) -> np.ndarray:
        """
        Compute distance from each grid point (px,py) to a 2D polyline (piecewise segments).
        Vectorized over grid; loops over segments (small).
        """
        dmin = np.full_like(px, np.inf, dtype=float)
        for k in range(len(pts) - 1):
            ax, ay = float(pts[k, 0]), float(pts[k, 1])
            bx, by = float(pts[k + 1, 0]), float(pts[k + 1, 1])
            abx, aby = bx - ax, by - ay
            denom = abx * abx + aby * aby
            if denom < 1e-9:
                continue
            t = ((px - ax) * abx + (py - ay) * aby) / denom
            t = np.clip(t, 0.0, 1.0)
            cx = ax + t * abx
            cy = ay + t * aby
            d = np.sqrt((px - cx) ** 2 + (py - cy) ** 2)
            dmin = np.minimum(dmin, d)
        return dmin

    def _polyline_length(pts: np.ndarray) -> float:
        seg = pts[1:] - pts[:-1]
        return float(np.sum(np.sqrt(np.sum(seg * seg, axis=1))))

    # ---------------------------------------------------------------------
    # Peaks (gaussian bumps)
    # ---------------------------------------------------------------------
    base_min, base_max = float(base_size_range[0]), float(base_size_range[1])
    for _ in range(int(num_peaks)):
        amp = float(np.random.uniform(*height_range))
        base = float(np.random.uniform(base_min, base_max))
        # Map base size to gaussian sigma. Smaller sigma => steeper.
        sigma = max(0.5, 0.45 * base)

        x0 = float(np.random.uniform(0.0, float(space_dim[0])))
        y0 = float(np.random.uniform(0.0, float(space_dim[1])))

        r2 = (X - x0) ** 2 + (Y - y0) ** 2
        H += amp * np.exp(-r2 / (2.0 * sigma**2))

    # ---------------------------------------------------------------------
    # Ridges (elongated features along random lines)
    # ---------------------------------------------------------------------
    ridge_count = int(max(0, ridge_count))
    for _ in range(ridge_count):
        amp = float(np.random.uniform(*height_range))
        sigma = max(1.0, 0.35 * float(np.random.uniform(base_min, base_max)))

        # Random line defined by point + direction
        x0 = float(np.random.uniform(0.0, float(space_dim[0])))
        y0 = float(np.random.uniform(0.0, float(space_dim[1])))
        theta = float(np.random.uniform(0.0, 2.0 * np.pi))
        nx = -np.sin(theta)
        ny = np.cos(theta)

        # Distance from each point to the line (signed)
        dist = (X - x0) * nx + (Y - y0) * ny
        H += 0.8 * amp * np.exp(-(dist**2) / (2.0 * sigma**2))

    # ---------------------------------------------------------------------
    # Ridge chains: finite continuous mountains formed by polylines + peaks along them
    # ---------------------------------------------------------------------
    ridge_chain_count = int(max(0, ridge_chain_count))
    ridge_chain_points = int(max(2, ridge_chain_points))
    chain_sigma = float(max(1.0, ridge_chain_width))
    chain_spacing = float(max(1.0, ridge_chain_peak_spacing))
    chain_boost = float(max(0.0, ridge_chain_peak_boost))

    for _ in range(ridge_chain_count):
        # Create a polyline that spans across the map (start/end on random edges)
        margin = 0.05
        sx = float(np.random.uniform(0.0, float(space_dim[0]) * margin))
        sy = float(np.random.uniform(0.0, float(space_dim[1]) * margin))
        ex = float(np.random.uniform(float(space_dim[0]) * (1.0 - margin), float(space_dim[0])))
        ey = float(np.random.uniform(float(space_dim[1]) * (1.0 - margin), float(space_dim[1])))

        pts = np.zeros((ridge_chain_points, 2), dtype=float)
        pts[0] = [sx, sy]
        pts[-1] = [ex, ey]

        # Intermediate points with some randomness (gives curvature)
        for i in range(1, ridge_chain_points - 1):
            t = i / (ridge_chain_points - 1)
            mx = (1.0 - t) * sx + t * ex
            my = (1.0 - t) * sy + t * ey
            jitter = 0.18
            jx = float(np.random.uniform(-jitter, jitter)) * float(space_dim[0])
            jy = float(np.random.uniform(-jitter, jitter)) * float(space_dim[1])
            pts[i] = [np.clip(mx + jx, 0.0, float(space_dim[0])), np.clip(my + jy, 0.0, float(space_dim[1]))]

        # Ridge "body": gaussian band around the polyline
        dist = _distance_to_polyline(X, Y, pts)
        ridge_amp = float(np.random.uniform(*height_range))
        H += 0.9 * ridge_amp * np.exp(-(dist**2) / (2.0 * chain_sigma**2))

        # Peaks along the chain: sample points along polyline length
        L = _polyline_length(pts)
        if L <= 1e-6:
            continue
        n_peaks = int(max(2, np.floor(L / chain_spacing)))

        # Generate peak centers by walking along segments
        segs = pts[1:] - pts[:-1]
        seg_lens = np.sqrt(np.sum(segs * segs, axis=1))
        cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
        samples = np.linspace(0.0, cum[-1], n_peaks)

        for s_dist in samples:
            k = int(np.searchsorted(cum, s_dist, side="right") - 1)
            k = int(np.clip(k, 0, len(seg_lens) - 1))
            if seg_lens[k] < 1e-6:
                continue
            u = (s_dist - cum[k]) / seg_lens[k]
            p = pts[k] + u * segs[k]

            amp = chain_boost * float(np.random.uniform(*height_range))
            base = float(np.random.uniform(base_min, base_max))
            sigma = max(0.8, 0.6 * base)
            r2 = (X - float(p[0])) ** 2 + (Y - float(p[1])) ** 2
            H += amp * np.exp(-r2 / (2.0 * sigma**2))

    # ---------------------------------------------------------------------
    # Pits / valleys ("hollow gaps")
    # ---------------------------------------------------------------------
    pit_count = int(max(0, pit_count))
    for _ in range(pit_count):
        depth = float(np.random.uniform(*pit_depth_range))
        sigma = max(1.0, 0.55 * float(np.random.uniform(base_min, base_max)))
        x0 = float(np.random.uniform(0.0, float(space_dim[0])))
        y0 = float(np.random.uniform(0.0, float(space_dim[1])))
        r2 = (X - x0) ** 2 + (Y - y0) ** 2
        H -= depth * np.exp(-r2 / (2.0 * sigma**2))

    H = np.clip(H, 0.0, None)

    # Clear out area around start position
    dist_start = np.sqrt((X - float(start_pos[0])) ** 2 + (Y - float(start_pos[1])) ** 2)
    H[dist_start < float(min_start_distance)] = 0.0

    # Make mountains more aggressive: nonlinear contrast
    hmax = float(H.max())
    if hmax > 1e-6:
        s = float(max(0.5, steepness))
        H = (H / hmax) ** s * hmax

    # Cap to the configured Z extent (keeps everything inside the box)
    H = np.clip(H, 0.0, float(space_dim[2]))

    # Store as a continuous surface (mesh/grid)
    # Note: keep the start area carved-out as flat (done above).
    obstacles.append(HeightFieldTerrain(xs=xs, ys=ys, heights=H))

    # Add some smaller rocks (cylinders) on top of the surface / valleys
    num_rocks = int(max(0, int(num_peaks * 0.35)))
    for _ in range(num_rocks):
        radius = float(np.random.uniform(0.5, 2.0))
        height = float(np.random.uniform(2.0, 10.0))

        x = float(np.random.uniform(radius, float(space_dim[0]) - radius))
        y = float(np.random.uniform(radius, float(space_dim[1]) - radius))
        center = np.array([x, y, 0.0], dtype=float)

        if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + float(min_start_distance)):
            continue

        obstacles.append(CylinderObstacle(center=center, height=height, radius=radius))

    return obstacles


def generate_mountain_ridge(space_dim, ridge_length=50.0, ridge_width=5.0,
                            peak_height_range=(15.0, 25.0), num_peaks=5):
    """
    Generate a continuous mountain ridge.
    
    Args:
        space_dim: Dimensions of the 3D space
        ridge_length: Length of the ridge
        ridge_width: Width of the ridge
        peak_height_range: (min, max) height for peaks along ridge
        num_peaks: Number of peaks along the ridge
        
    Returns:
        list: List of box obstacles forming a ridge
    """
    # Legacy helper kept for backwards compatibility with older demos.
    from .obstacles import BoxObstacle

    obstacles = []
    
    # Ridge direction (diagonal across space)
    start_x = space_dim[0] * 0.2
    start_y = space_dim[1] * 0.2
    end_x = space_dim[0] * 0.8
    end_y = space_dim[1] * 0.8
    
    dx = (end_x - start_x) / num_peaks
    dy = (end_y - start_y) / num_peaks
    
    for i in range(num_peaks):
        x = start_x + i * dx
        y = start_y + i * dy
        height = np.random.uniform(*peak_height_range)
        
        center = np.array([x, y, 0])
        size = np.array([ridge_width, ridge_width, height])
        
        obstacles.append(BoxObstacle(center=center, size=size))
    
    return obstacles
