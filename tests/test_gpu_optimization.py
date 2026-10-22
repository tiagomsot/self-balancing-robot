"""Numerical acquisition and safety-ranking checks without a CUDA dependency."""
import unittest
import numpy as np
from gpu.optimization_core import aggregate, expected_improvement, score_worlds


class OptimizationTests(unittest.TestCase):
    def test_ei_favors_lower_predictions_when_certain(self):
        np.testing.assert_allclose(expected_improvement([1, 2, 3], [0, 0, 0], 2, 0), [1, 0, 0])

    def test_ei_values_uncertainty_at_the_incumbent(self):
        values = expected_improvement([2, 2], [0, 1], 2, 0)
        self.assertEqual(values[0], 0)
        self.assertAlmostEqual(values[1], 1/np.sqrt(2*np.pi))

    def test_worst_case_prevents_mean_only_selection(self):
        # Equal means, but concentrated poor responses receive a larger cost.
        scores = aggregate([[2, 2], [0, 4]])
        self.assertLess(scores[0], scores[1])

    def test_faults_current_and_nonfinite_states_cannot_pass(self):
        stats, diagnostic, extra = np.zeros((5, 8)), np.zeros((5, 4)), np.zeros((5, 5))
        diagnostic[1, 3] = 1
        diagnostic[2, 0] = .61
        diagnostic[3, 1] = 1.21
        costs, passed = score_worlds(stats, diagnostic, extra, [True]*4+[False], 8)
        np.testing.assert_array_equal(passed, [True, False, False, False, False])
        self.assertTrue(np.all(costs[1:] >= costs[0]+1000))

    def test_nonfinite_metric_is_rejected_even_with_finite_state_flag(self):
        stats, diagnostic, extra = np.zeros((1, 8)), np.zeros((1, 4)), np.zeros((1, 5))
        stats[0, 1] = np.nan
        costs, passed = score_worlds(stats, diagnostic, extra, [True], 8)
        self.assertFalse(passed[0])
        self.assertTrue(np.isfinite(costs[0]))

    def test_drift_is_logged_without_demanding_position_or_speed_hold(self):
        stats, diagnostic, extra = np.zeros((2, 8)), np.zeros((2, 4)), np.zeros((2, 5))
        stats[1,2:4]=[100,100];stats[1,6:8]=[20,3];extra[1,3:5]=[100,100]
        costs, passed = score_worlds(stats,diagnostic,extra,[True,True],8)
        np.testing.assert_array_equal(passed,[True,True])
        self.assertEqual(costs[0],costs[1])


if __name__ == "__main__":
    unittest.main()
