import argparse
import csv
import re
from pathlib import Path
import tempfile
import unittest

import run_trial


def default_args(**changes):
    values = dict(
        swing_height=None, mpc_frequency=None, sqp_iterations=None,
        robust="keep", robust_t_a=None, robust_t_b=None, robust_d=None,
        robust_hard_boundary_start="keep", robust_hard_boundary_end="keep",
        robust_slack_boundary_start="keep", robust_slack_boundary_end="keep",
        robust_slack_weight_start=None, robust_slack_weight_end=None,
        robust_splice="keep", robust_verbose="keep",
    )
    values.update(changes)
    return argparse.Namespace(**values)


class TrialTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.task_text = run_trial.SOURCE_TASK_INFO_PATH.read_text(encoding="utf-8")

    def test_keep_preserves_task_file(self):
        overrides = run_trial.task_info_overrides(default_args())
        self.assertEqual(overrides, {})
        self.assertEqual(run_trial.render_task_info(self.task_text, overrides), self.task_text)
        robust = run_trial.effective_task_parameters(self.task_text)["robustPhase"]
        duration = float(robust["t_a"]) + float(robust["t_b"])
        self.assertAlmostEqual(robust["T_robust_s"], duration)
        self.assertAlmostEqual(robust["v_max"], 2 * float(robust["d_max"]) / duration)
        self.assertNotIn("P", robust)

    def test_on_off_overrides_and_derived_velocity(self):
        for mode in ("on", "off"):
            overrides = run_trial.task_info_overrides(default_args(
                robust=mode, robust_t_a=0.03, robust_t_b=0.07, robust_d=0.04,
            ))
            self.assertNotIn("robustPhase.P", overrides)
            self.assertNotIn("robustPhase.v_max", overrides)
            rendered = run_trial.render_task_info(self.task_text, overrides)
            robust = run_trial.effective_task_parameters(rendered)["robustPhase"]
            self.assertEqual(robust["enabled"], "true" if mode == "on" else "false")
            self.assertAlmostEqual(float(robust["t_a"]), 0.03)
            self.assertAlmostEqual(float(robust["t_b"]), 0.07)
            self.assertAlmostEqual(robust["v_max"], 1.0)  # d_max, independent of seed d

    def test_invalid_offsets_and_width(self):
        for key in ("robust_t_a", "robust_t_b", "robust_d"):
            for value in (-0.01, float("nan"), float("inf")):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    run_trial.task_info_overrides(default_args(**{key: value}))
        overrides = run_trial.task_info_overrides(default_args(robust_t_a=0, robust_t_b=0))
        with self.assertRaises(ValueError):
            run_trial.effective_task_parameters(run_trial.render_task_info(self.task_text, overrides))
        for key in ("t_a", "t_b", "d", "d_min", "d_max"):
            invalid = run_trial.replace_task_value(self.task_text, key, "nan", "robustPhase")
            with self.assertRaises(ValueError):
                run_trial.effective_task_parameters(invalid)

    def test_width_reward_configuration(self):
        for value in ("0", "10", "1000"):
            text = run_trial.replace_task_value(self.task_text, "w_d", value, "robustPhase")
            robust = run_trial.effective_task_parameters(text)["robustPhase"]
            self.assertEqual(robust["w_d"], value)
        for value in ("-1", "nan", "inf"):
            text = run_trial.replace_task_value(self.task_text, "w_d", value, "robustPhase")
            with self.assertRaises(ValueError):
                run_trial.effective_task_parameters(text)
        legacy = re.sub(r"^  w_d\s+.*\n", "", self.task_text, flags=re.MULTILINE)
        self.assertEqual(run_trial.effective_task_parameters(legacy)["robustPhase"]["w_d"], "0.0")

    def test_summary_new_and_legacy_headers(self):
        result = {
            "experiment": {
                "trial": "timing_test", "tag": "test", "terrain": "test_scene",
                "mode": "perceptive_dev_v2", "terrain_z_offset": 0.0,
                "effective_task_parameters": run_trial.effective_task_parameters(self.task_text),
            },
            "success": True,
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current = root / "summary.csv"
            run_trial.append_trial_summary(current, result, root / "result.json")
            with current.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertIn("robust_t_a", rows[0])
            self.assertIn("robust_t_b", rows[0])
            self.assertIn("robust_w_d", rows[0])
            self.assertNotIn("robust_P", rows[0])
            legacy = root / "legacy.csv"
            legacy.write_text("trial,robust_P,robust_v_max\nold,10,0.6\n")
            run_trial.append_trial_summary(legacy, result, root / "result.json")
            with legacy.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["robust_P"], "10")
            self.assertEqual(rows[1]["robust_P"], "")


if __name__ == "__main__":
    unittest.main()
