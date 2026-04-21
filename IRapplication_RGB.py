import numpy as np
#import quaternion
# Unfortunately, the quaternion module refuses to install
import os
from plyfile import PlyData, PlyElement
from PIL import Image
from datetime import datetime
from scipy.spatial.transform import Rotation as R

#FOV, degrees
FOVhoz = 45
FOVver = 37
q_camera = np.array([0.5, 0, 0, 0.866])
#q_camera_norm = (q_camera - q_camera.min()) / (q_camera.max() - q_camera.min())
# camera_q = qx qy qz qw rotation from lidar frame
ply_data = PlyData.read("calibrationtest.PLY")
tiff_dir = "ImageData"
posefile = "traj_lidar.txt"
#stamp x y z qx qy qz qw

vertex_stack = np.stack([
    ply_data['vertex'].data['x'],
    ply_data['vertex'].data['y'],
    ply_data['vertex'].data['z'],
    np.zeros(ply_data['vertex'].data['x'].size),  # R
    np.zeros(ply_data['vertex'].data['x'].size),  # G
    np.zeros(ply_data['vertex'].data['x'].size)   # B
], axis=-1)

#print(vertex_array[0])

pose = np.loadtxt(posefile)

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([x, y, z, w])

def tiff_unix_time(tiff_page):
    EXIF_TAG_CODE = 34665  # ExifTag

    if EXIF_TAG_CODE not in tiff_page.tags:
        return None

    exif_dict = tiff_page.tags[EXIF_TAG_CODE].value
    dt_str = exif_dict.get('DateTimeOriginal')
    subseconds = exif_dict.get('SubsecTimeOriginal', '')

    if not dt_str:
        print(dt_str)
        print("Failed to extrat timestamp!")
        return None

    try:
        # Parse main datetime part
        dt = datetime.strptime(dt_str, "%Y:%m:%d %H:%M:%S")

        # If subseconds present, convert to microseconds and add
        if subseconds.isdigit():
            microsec = int(subseconds.ljust(6, '0'))  # pad to microseconds
            dt = dt.replace(microsecond=microsec)

        return dt.timestamp()
    except Exception as e:
        print(f"Error parsing datetime with subseconds: {e}")
        return None

def posefromtime(posematrix, timestamp):
    # Extract times
    times = posematrix[:,0]
    # Subtraction
    times_dist = np.absolute(times - timestamp)
    # Get least deviation
    i = np.argmin(times_dist)

    return posematrix[i]


def apply_pixel_colors_to_vertices_vectorized(
    image,
    height,
    width,
    q_image,
    pos,
    FOVhoz,
    FOVver,
    Verticies,
    K
):
    """
    Vectorized version of color assignment:
    Assigns pixel colors to K nearest projected vertices within each pixel.
    """

    # --- 1. Camera rotation and intrinsics ---
    R_cam = R.from_quat(q_image).as_matrix()
    fx = (width / 2) / np.tan(np.deg2rad(FOVhoz) / 2)
    fy = (height / 2) / np.tan(np.deg2rad(FOVver) / 2)

    # --- 2. Transform vertices into camera coordinates ---
    verts_world = Verticies[:, :3]  # (N,3)
    verts_cam = (R_cam.T @ (verts_world - pos).T) # (N,3)
    verts_cam = verts_cam.T
    #print(verts_world)
    #print(verts_world - pos)

    x, y, z = verts_cam[:, 0], verts_cam[:, 1], verts_cam[:, 2]
    valid = z > 0
    verts_cam = verts_cam[valid]
    idx_valid = np.where(valid)[0]
    #print(idx_valid)

    # --- 3. Project into pixel coordinates ---
    u = (fx * (x[valid] / z[valid])) + width / 2
    v = (fy * (y[valid] / z[valid])) + height / 2

    # --- 4. Filter points within the image ---
    in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v = u[in_bounds], v[in_bounds]
    idx_valid = idx_valid[in_bounds]

    # --- 5. Round to pixel indices ---
    px = np.floor(u).astype(int)
    py = np.floor(v).astype(int)
    pixel_indices = py * width + px  # flatten pixel index

    # --- 6. Sort vertices by pixel and distance ---
    # Compute projected distance from pixel center
    du = u - (px + 0.5)
    dv = v - (py + 0.5)
    dist2 = du**2 + dv**2

    order = np.lexsort((dist2, pixel_indices))
    pixel_indices = pixel_indices[order]
    idx_valid = idx_valid[order]
    dist2 = dist2[order]

    # --- 7. For each pixel, take up to K nearest vertices ---
    # Find unique pixels and their start indices
    unique_pixels, start_idx, counts = np.unique(pixel_indices, return_index=True, return_counts=True)

    # Limit K per pixel
    keep_mask = np.zeros_like(pixel_indices, dtype=bool)
    for i, count in enumerate(counts):
        start = start_idx[i]
        end = start + min(count, K)
        keep_mask[start:end] = True

    selected_vertices = idx_valid[keep_mask]
    selected_pixels = pixel_indices[keep_mask]

    # --- 8. Assign pixel colors to selected vertices ---
    colors = Verticies[:, 3:6].copy()  # (N,3)

    pixel_colors = image.reshape(-1, 3)[selected_pixels]  # (M,3)

    colors[selected_vertices] = pixel_colors

    Verticies[:, 3:6] = colors

    # --- 9. Return updated vertices ---
    #Verticies[:, 3] = colors
    return Verticies

def save_ply_grayscale_as_rgb(vertices, filename):
    """
    Save XYZ + grayscale as standard RGB .ply (appears gray in viewers).
    vertices: (N, 4) array [x, y, z, gray]
    """

    n = vertices.shape[0]

    # Create structured array
    data = np.zeros(n, dtype=[
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
    ])

    rgb = vertices[:, 3:6].astype(np.uint8)

    data['red'] = rgb[:, 0]
    data['green'] = rgb[:, 1]
    data['blue'] = rgb[:, 2]
    data['x'], data['y'], data['z'] = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    #data['red'] = data['green'] = data['blue'] = gray

    el = PlyElement.describe(data, 'vertex')
    PlyData([el], text=True).write(filename)
    print(f"Saved {vertices.shape[0]} vertices with RGB color to {filename}")

def filename_to_unix(filename):
    # Remove extension
    name = os.path.splitext(filename)[0]
    
    # Split parts
    parts = name.split('_')
    
    # Extract date, time, milliseconds
    date_part = parts[0]        # YYYYMMDD
    time_part = parts[1]        # HHMMSS
    millis_part = parts[2]      # milliseconds
    
    # Combine into datetime string
    dt_str = f"{date_part}{time_part}{millis_part}"
    
    # Parse into datetime object
    dt = datetime.strptime(dt_str, "%Y%m%d%H%M%S%f")
    
    # Convert to Unix timestamp (seconds)
    unix_time = dt.timestamp()
    
    return unix_time

for filename in os.listdir(tiff_dir):
    if filename.lower().endswith((".jpg", ".jpeg")):
        filepath = os.path.join(tiff_dir, filename)
        print(f"\nProcessing file: {filename}")

        # Load image (RGB)
        image = Image.open(filepath).convert("RGB")
        pixels = np.array(image)  # shape (H, W, 3)
        height, width, _ = pixels.shape

        # NOTE: Generate timestamp from filename and convert to unix time
        timestamp = filename_to_unix(filename)

        pagepose = posefromtime(pose, timestamp)

        pageposition = np.array([pagepose[1], pagepose[2], pagepose[3]])

        q_lidar = np.array([pagepose[4], pagepose[5], pagepose[6], pagepose[7]])
        q_image = quaternion_multiply(q_camera, q_lidar)

        vertex_stack = apply_pixel_colors_to_vertices_vectorized(
            pixels, height, width,
            q_image, pageposition,
            FOVhoz, FOVver,
            vertex_stack, 1000
        )

save_ply_grayscale_as_rgb(vertex_stack, "pointmapCOLORED.ply")

#print("Execution completed")

#For each tiff frame
    #Get timestamp
    #Find pose associated with timestamp
    #For each pixel
        #Isolate points within pixel cone
        #Apply color to nearest group
        #Apply "colored" flag to verticies
