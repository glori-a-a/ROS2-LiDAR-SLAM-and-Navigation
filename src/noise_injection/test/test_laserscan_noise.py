import unittest

import numpy as np

from noise_injection.noise import apply_scan_noise


class FixedNoiseRng:
    def normal(self, mean, stddev, size):
        return np.array([-10.0, 10.0])

    def random(self, size):
        return np.ones(size)


class FixedDropoutRng:
    def normal(self, mean, stddev, size):
        return np.zeros(size)

    def random(self, size):
        return np.array([0.0, 0.8, 0.2])


class TestLaserScanNoise(unittest.TestCase):
    def test_zero_noise_preserves_ranges(self):
        ranges = np.array([1.0, 2.0, np.inf, np.nan, 20.0])

        result = apply_scan_noise(
            ranges,
            range_min=0.1,
            range_max=10.0,
            range_noise_std=0.0,
            dropout_probability=0.0,
            rng=np.random.default_rng(123),
        )

        np.testing.assert_array_equal(result, ranges)

    def test_deterministic_seed(self):
        ranges = np.linspace(0.5, 4.5, 20)

        first = apply_scan_noise(
            ranges,
            range_min=0.1,
            range_max=5.0,
            range_noise_std=0.05,
            dropout_probability=0.25,
            rng=np.random.default_rng(42),
        )
        second = apply_scan_noise(
            ranges,
            range_min=0.1,
            range_max=5.0,
            range_noise_std=0.05,
            dropout_probability=0.25,
            rng=np.random.default_rng(42),
        )

        np.testing.assert_array_equal(first, second)

    def test_range_clipping(self):
        result = apply_scan_noise(
            [0.2, 9.9],
            range_min=0.1,
            range_max=10.0,
            range_noise_std=1.0,
            dropout_probability=0.0,
            rng=FixedNoiseRng(),
        )

        np.testing.assert_allclose(result, [0.1, 10.0])

    def test_dropout_sets_valid_beams_to_infinity(self):
        result = apply_scan_noise(
            [1.0, 2.0, np.inf, 3.0],
            range_min=0.1,
            range_max=10.0,
            range_noise_std=0.0,
            dropout_probability=0.5,
            rng=FixedDropoutRng(),
        )

        self.assertTrue(np.isinf(result[0]))
        self.assertEqual(result[1], 2.0)
        self.assertTrue(np.isinf(result[2]))
        self.assertTrue(np.isinf(result[3]))


if __name__ == "__main__":
    unittest.main()
