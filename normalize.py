import cv2
import numpy as np
from pathlib import Path
import argparse


def find_tiff_images(input_dir):
    """Return TIFF images sorted by filename."""
    extensions = {".tif", ".tiff", ".TIFF"}

    files = [
        p for p in Path(input_dir).iterdir()
        if p.is_file() and p.suffix.lower() in extensions
    ]

    return sorted(files)


def read_16bit_grayscale(path):
    """Read a TIFF image and verify that it is 16-bit grayscale."""
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)

    if image is None:
        raise ValueError(f"Could not read image: {path}")

    if image.dtype != np.uint16:
        raise ValueError(
            f"{path.name}: expected uint16 image, got {image.dtype}"
        )

    if image.ndim != 2:
        raise ValueError(
            f"{path.name}: expected grayscale image, "
            f"got shape {image.shape}"
        )

    return image


def calculate_normalization_range(
    images,
    lower_percentile=0.0,
    upper_percentile=100.0
):
    """
    Calculate a global normalization range across all images.

    Percentiles can be used to reject extreme outliers. For example:
        lower_percentile=1
        upper_percentile=99
    """

    all_values = []

    for image in images:
        all_values.append(image.ravel())

    all_values = np.concatenate(all_values)

    lower = np.percentile(all_values, lower_percentile)
    upper = np.percentile(all_values, upper_percentile)

    return float(lower), float(upper)


def normalize_image(image, norm_min, norm_max):
    """
    Normalize a 16-bit image to 8-bit [0, 255].
    Values outside the normalization range are clipped.
    """

    if norm_max <= norm_min:
        raise ValueError(
            f"Invalid normalization range: {norm_min} - {norm_max}"
        )

    image_float = image.astype(np.float32)

    # Clip to the selected thermal range.
    image_float = np.clip(image_float, norm_min, norm_max)

    # Scale to 0-255.
    normalized = (
        (image_float - norm_min)
        / (norm_max - norm_min)
        * 255.0
    )

    return np.round(normalized).astype(np.uint8)


def process_folder(
    input_dir,
    output_dir,
    lower_percentile=0.0,
    upper_percentile=100.0
):
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)

    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = find_tiff_images(input_dir)

    if not image_paths:
        raise RuntimeError(
            f"No TIFF images found in {input_dir}"
        )

    print(f"Found {len(image_paths)} TIFF images.")

    # ------------------------------------------------------------------
    # Read all images first.
    # ------------------------------------------------------------------
    images = []
    image_ranges = []

    for path in image_paths:
        print(f"Reading: {path.name}")

        image = read_16bit_grayscale(path)
        images.append(image)

        original_min = int(image.min())
        original_max = int(image.max())

        image_ranges.append({
            "filename": path.name,
            "min": original_min,
            "max": original_max,
            "mean": float(image.mean())
        })

    # ------------------------------------------------------------------
    # Calculate one global normalization range for the entire sequence.
    # ------------------------------------------------------------------
    norm_min, norm_max = calculate_normalization_range(
        images,
        lower_percentile=lower_percentile,
        upper_percentile=upper_percentile
    )

    print()
    print("Global normalization range:")
    print(f"  Minimum: {norm_min:.3f}")
    print(f"  Maximum: {norm_max:.3f}")
    print()

    # ------------------------------------------------------------------
    # Normalize and save.
    # ------------------------------------------------------------------
    for index, (path, image) in enumerate(zip(image_paths, images), start=1):

        normalized = normalize_image(
            image,
            norm_min,
            norm_max
        )

        output_name = f"{path.stem}_normalized.tiff"
        output_path = output_dir / output_name

        success = cv2.imwrite(
            str(output_path),
            normalized
        )

        if not success:
            raise RuntimeError(
                f"Failed to save {output_path}"
            )

        print(
            f"[{index:04d}/{len(image_paths):04d}] "
            f"{path.name} -> {output_name}"
        )

    # ------------------------------------------------------------------
    # Write metadata.
    # ------------------------------------------------------------------
    metadata_path = output_dir / "metadata.txt"

    with open(metadata_path, "w", encoding="utf-8") as f:

        f.write("THERMAL IMAGE NORMALIZATION METADATA\n")
        f.write("=" * 60 + "\n\n")

        f.write(f"Input folder: {input_dir.resolve()}\n")
        f.write(f"Output folder: {output_dir.resolve()}\n")
        f.write(f"Number of images: {len(image_paths)}\n\n")

        f.write("NORMALIZATION\n")
        f.write("-" * 60 + "\n")
        f.write("Output data type: uint8\n")
        f.write("Output range: 0 - 255\n")
        f.write(
            f"Lower percentile: {lower_percentile:.3f}\n"
        )
        f.write(
            f"Upper percentile: {upper_percentile:.3f}\n"
        )
        f.write(
            f"Global normalization minimum: {norm_min:.6f}\n"
        )
        f.write(
            f"Global normalization maximum: {norm_max:.6f}\n"
        )
        f.write("\n")

        f.write("IMAGE METADATA\n")
        f.write("-" * 60 + "\n")
        f.write(
            "Filename\tOriginal Min\tOriginal Max\tOriginal Mean\n"
        )

        for info in image_ranges:
            f.write(
                f"{info['filename']}\t"
                f"{info['min']}\t"
                f"{info['max']}\t"
                f"{info['mean']:.6f}\n"
            )

    print()
    print("Processing complete.")
    print(f"Normalized images: {output_dir}")
    print(f"Metadata: {metadata_path}")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Batch normalize 16-bit grayscale thermal TIFF images "
            "to 8-bit images suitable for computer vision."
        )
    )

    parser.add_argument(
        "input_dir",
        help="Folder containing the 16-bit thermal TIFF images."
    )

    parser.add_argument(
        "output_dir",
        help="Folder where normalized images and metadata will be saved."
    )

    parser.add_argument(
        "--lower-percentile",
        type=float,
        default=0.0,
        help=(
            "Lower percentile used for normalization. "
            "Default: 0 (no clipping)."
        )
    )

    parser.add_argument(
        "--upper-percentile",
        type=float,
        default=100.0,
        help=(
            "Upper percentile used for normalization. "
            "Default: 100 (no clipping)."
        )
    )

    args = parser.parse_args()

    if not 0 <= args.lower_percentile <= 100:
        parser.error("lower-percentile must be between 0 and 100.")

    if not 0 <= args.upper_percentile <= 100:
        parser.error("upper-percentile must be between 0 and 100.")

    if args.lower_percentile >= args.upper_percentile:
        parser.error(
            "lower-percentile must be smaller than upper-percentile."
        )

    process_folder(
        args.input_dir,
        args.output_dir,
        lower_percentile=args.lower_percentile,
        upper_percentile=args.upper_percentile
    )


if __name__ == "__main__":
    main()