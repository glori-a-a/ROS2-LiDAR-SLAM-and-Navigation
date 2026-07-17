import numpy as np


def apply_scan_noise(
    ranges,
    range_min,
    range_max,
    range_noise_std,
    dropout_probability,
    rng=None,
):
    """Return ranges with Gaussian noise and random dropouts applied.

    Only finite ranges already inside [range_min, range_max] are considered
    valid. Existing NaN, infinity, and out-of-range values are preserved.
    """
    if range_min > range_max:
        raise ValueError("range_min must be less than or equal to range_max")
    if range_noise_std < 0.0:
        raise ValueError("range_noise_std must be non-negative")
    if not 0.0 <= dropout_probability <= 1.0:
        raise ValueError("dropout_probability must be between 0.0 and 1.0")

    if rng is None:
        rng = np.random.default_rng()

    noisy_ranges = np.asarray(ranges, dtype=float).copy()
    valid_mask = (
        np.isfinite(noisy_ranges)
        & (noisy_ranges >= range_min)
        & (noisy_ranges <= range_max)
    )
    valid_count = int(np.count_nonzero(valid_mask))

    if valid_count == 0:
        return noisy_ranges

    if range_noise_std > 0.0:
        noise = np.asarray(
            rng.normal(0.0, range_noise_std, valid_count),
            dtype=float,
        )
        noisy_ranges[valid_mask] += noise

    noisy_ranges[valid_mask] = np.clip(
        noisy_ranges[valid_mask],
        range_min,
        range_max,
    )

    if dropout_probability > 0.0:
        dropped = np.asarray(rng.random(valid_count), dtype=float) < dropout_probability
        valid_indices = np.flatnonzero(valid_mask)
        noisy_ranges[valid_indices[dropped]] = np.inf

    return noisy_ranges
