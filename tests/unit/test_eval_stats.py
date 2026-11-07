"""Evaluation statistics against published values and independent implementations (scipy, numpy, simulation)."""

import numpy as np
import pytest

from robot_platform.evaluation import stats

scipy_stats = pytest.importorskip('scipy.stats')

# Newcombe (1998), Two-sided confidence intervals for the single proportion, Stat Med 17:857, Table I,
# method 3 (Wilson score, no continuity correction), plus the usual small-n endpoints.
NEWCOMBE = [
    ((81, 263), (0.2553, 0.3662)),
    ((15, 148), (0.0624, 0.1605)),
    ((0, 20), (0.0, 0.1611)),
    ((1, 29), (0.0061, 0.1718)),
]


@pytest.mark.parametrize(
    'counts, expected', NEWCOMBE + [((0, 10), (0.0, 0.2775)), ((10, 10), (0.7225, 1.0)), ((5, 10), (0.2366, 0.7634))]
)
def test_wilson_interval_matches_published_values(counts, expected):
    low, high = stats.wilson_interval(*counts)
    assert (round(low, 4), round(high, 4)) == expected


@pytest.mark.parametrize('successes, trials', [(0, 1), (1, 1), (3, 7), (12, 30), (49, 50), (0, 200)])
def test_wilson_interval_matches_scipy(successes, trials):
    reference = scipy_stats.binomtest(successes, trials).proportion_ci(confidence_level=0.95, method='wilson')
    assert stats.wilson_interval(successes, trials) == pytest.approx((reference.low, reference.high), abs=1e-12)


def test_wilson_interval_edges_and_bad_counts():
    assert stats.wilson_interval(0, 0) == (0.0, 1.0)  # No trials: no information.
    assert stats.wilson_interval(0, 5)[0] == 0.0 and stats.wilson_interval(5, 5)[1] == 1.0
    for bad in ((3, 2), (-1, 4), (1.5, 4)):
        with pytest.raises(ValueError):
            stats.wilson_interval(*bad)


@pytest.mark.parametrize(
    'a, b', [((3, 10), (9, 10)), ((7, 20), (15, 20)), ((40, 100), (55, 100)), ((0, 12), (5, 12)), ((18, 20), (18, 20))]
)
def test_two_proportion_test_matches_scipy(a, b):
    result = stats.two_proportion_test(*a, *b)
    table = [[a[0], a[1] - a[0]], [b[0], b[1] - b[0]]]
    # The pooled two-sided z-test is the uncorrected chi-square test of the 2x2 table.
    assert result['p_z_test'] == pytest.approx(scipy_stats.chi2_contingency(table, correction=False)[1], rel=1e-9)
    assert result['p_fisher_exact'] == pytest.approx(scipy_stats.fisher_exact(table)[1], rel=1e-9)


def test_two_proportion_verdict_uses_fisher_for_small_expected_counts():
    small = stats.two_proportion_test(3, 10, 9, 10)
    assert small['method'] == 'fisher_exact' and small['min_expected_count'] < 5
    assert small['p_value'] == small['p_fisher_exact'] and small['differ']
    large = stats.two_proportion_test(40, 100, 55, 100)
    assert large['method'] == 'z_test' and large['p_value'] == large['p_z_test'] and large['differ']
    same = stats.two_proportion_test(5, 10, 5, 10)
    assert not same['differ'] and same['p_value'] == 1.0
    assert not stats.two_proportion_test(0, 10, 0, 10)['differ']
    with pytest.raises(ValueError):
        stats.two_proportion_test(1, 0, 1, 2)


def test_trials_per_arm_matches_textbook_values():
    # Pooled-variance formula without continuity correction: 50% vs 60% needs 388 per group and 20% vs 40% 82
    # (the standard tabulated values, e.g. Fleiss, Levin and Paik).
    assert stats.trials_per_arm(0.5, 0.6) == 388
    assert stats.trials_per_arm(0.2, 0.4) == 82
    assert stats.trials_per_arm(0.6, 0.5) == 388  # Symmetric.
    assert stats.trials_per_arm(0.2, 0.4, power=0.9) > 82 and stats.trials_per_arm(0.2, 0.4, alpha=0.01) > 82
    with pytest.raises(ValueError):
        stats.trials_per_arm(0.5, 0.5)


@pytest.mark.parametrize('p1, p2', [(0.5, 0.6), (0.2, 0.4)])
def test_trials_per_arm_reaches_eighty_percent_power_by_simulation(p1, p2):
    n = stats.trials_per_arm(p1, p2)
    rng = np.random.default_rng(20261016)
    reps = 20000
    a, b = rng.binomial(n, p1, reps), rng.binomial(n, p2, reps)
    pooled = (a + b) / (2 * n)
    z = (b - a) / n / np.sqrt(pooled * (1 - pooled) * 2 / n)
    power = np.mean(np.abs(z) > 1.959963984540054)
    assert power == pytest.approx(0.8, abs=0.02)


def test_percentile_matches_numpy_linear_interpolation():
    values = [12.1, 9.8, 10.4, 15.0, 11.2, 30.5, 10.0]
    for q in (0, 10, 50, 90, 100):
        assert stats.percentile(values, q) == pytest.approx(np.percentile(values, q))
    assert stats.percentile([], 50) is None and stats.percentile([4.0], 90) == 4.0
