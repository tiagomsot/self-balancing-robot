"""Budget and paired-initialization checks using cheap analytic objectives."""
import unittest
import numpy as np

from gpu.search_comparison import initial_design, run_search, select_candidate
from analysis.finalize_optimizer_comparison import choose_method


class SearchComparisonTests(unittest.TestCase):
    def setUp(self):
        self.initial = initial_design(42, [.7]*4)
        costs = np.sum((self.initial-.25)**2, axis=1)
        self.records = [dict(trial=i, stage="shared_initial_sobol", gains=p.tolist(),
                            objective=float(c), all_training_cases_pass=True, cases=[])
                        for i, (p,c) in enumerate(zip(self.initial, costs))]
        self.trace = [dict(evaluations=16, elapsed_s=.1, best_objective=min(costs))]
        self.evaluated = []
        self.proposed_counts = []

    def evaluate(self, points):
        self.evaluated.extend(np.array(points).tolist())
        self.assertTrue(np.all((points >= 0) & (points <= 1)))
        return np.sum((points-.25)**2, axis=1), np.ones(len(points), bool), [[]]*len(points), points

    def propose(self, observed, costs, count, rng, log):
        self.proposed_counts.append(len(observed))
        return rng.uniform(0, 1, (count, 4))

    def run_method(self, method, trials=64):
        return run_search(method, self.initial, self.records, self.trace, .1,
                          self.evaluate, self.propose, 42, trials=trials, batch_size=8)

    def test_initial_design_reproducible_and_contains_incumbent(self):
        np.testing.assert_array_equal(self.initial, initial_design(42, [.7]*4))
        np.testing.assert_array_equal(self.initial[0], [.7]*4)
        self.assertFalse(np.array_equal(self.initial, initial_design(7, [.7]*4)))

    def test_de_spends_exact_budget_and_reuses_all_initial_observations(self):
        result = self.run_method("differential_evolution")
        self.assertEqual(len(result["records"]), 64)
        self.assertEqual(len(self.evaluated), 48)
        self.assertEqual(result["records"][:16], self.records)
        self.assertEqual(result["optimizer_settings"]["generations"], 3)
        self.assertLessEqual(result["selected"]["objective"], min(r["objective"] for r in self.records))

    def test_bo_receives_accumulated_data_and_same_budget(self):
        result = self.run_method("bayesian")
        self.assertEqual(len(self.evaluated), 48)
        self.assertEqual(self.proposed_counts, [16,24,32,40,48,56])
        self.assertEqual(result["records"][:16], self.records)
        self.assertEqual([row["evaluations"] for row in result["trace"]], [16,24,32,40,48,56,64])

    def test_methods_cannot_mutate_shared_initial_observations(self):
        original = [row.copy() for row in self.records]
        first = self.run_method("bayesian")
        first["records"][0]["gains"][0] = -10
        self.run_method("differential_evolution")
        self.assertEqual(self.records, original)
        self.assertEqual(self.records[0]["gains"][0], .7)

    def test_selection_rejects_lower_scoring_failed_controller(self):
        records = [dict(objective=.1, all_training_cases_pass=False),
                   dict(objective=.2, all_training_cases_pass=True)]
        self.assertEqual(select_candidate(records)["objective"], .2)

    def test_partial_generation_budget_is_rejected(self):
        with self.assertRaises(ValueError): self.run_method("differential_evolution", 63)

    def test_finalists_rank_by_training_score_after_validation(self):
        results = {"bayesian":dict(training_score=.1, gpu_passed=True, gazebo_balance_passed=True,
                                    movement_passed=True, holdout_score=.3),
                   "differential_evolution":dict(training_score=.2, gpu_passed=True,
                                                gazebo_balance_passed=True, movement_passed=True, holdout_score=.1)}
        self.assertEqual(choose_method(results, .5), "bayesian")
        results["bayesian"]["movement_passed"] = False
        self.assertEqual(choose_method(results, .5), "differential_evolution")
        self.assertIsNone(choose_method(results, .15))

    def test_no_finalist_can_apply_without_gpu_and_gazebo_passes(self):
        results = {"bayesian":dict(training_score=.1, gpu_passed=False,
                                    gazebo_balance_passed=True, movement_passed=None)}
        self.assertIsNone(choose_method(results, .5))
        results["bayesian"].update(gpu_passed=True, gazebo_balance_passed=False)
        self.assertIsNone(choose_method(results, .5))


if __name__ == "__main__":
    unittest.main()
