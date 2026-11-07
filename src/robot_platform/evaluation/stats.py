"""Small-sample statistics for task success rates. Standard library only.

Robot trial counts are small (10-50), so every rate carries its interval and every
comparison says how many trials it would take to tell two policies apart.
"""

from __future__ import annotations

import math
from statistics import NormalDist

_NORMAL = NormalDist()


def _z(confidence):
    if not 0 < confidence < 1:
        raise ValueError('confidence must be in (0, 1)')
    return _NORMAL.inv_cdf(0.5 + confidence / 2)


def _counts(successes, trials):
    if type(successes) is not int or type(trials) is not int or trials < 0 or not 0 <= successes <= trials:
        raise ValueError('successes and trials must be integers with 0 <= successes <= trials')


def wilson_interval(successes, trials, confidence=0.95):
    """Wilson score interval for a binomial proportion (Wilson 1927; Newcombe 1998 method 3).

    Unlike the normal (Wald) interval it stays inside [0, 1] and does not collapse to zero
    width at 0/n or n/n, which is where small robot trial counts usually sit.
    """
    _counts(successes, trials)
    if trials == 0:
        return (0.0, 1.0)
    z = _z(confidence)
    p = successes / trials
    denominator = 1 + z * z / trials
    center = (p + z * z / (2 * trials)) / denominator
    half = z * math.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denominator
    # Exact endpoints at 0/n and n/n, without rounding residue.
    low = 0.0 if successes == 0 else max(0.0, center - half)
    high = 1.0 if successes == trials else min(1.0, center + half)
    return (low, high)


def _hypergeometric(k, row1, col1, total):
    return math.comb(col1, k) * math.comb(total - col1, row1 - k) / math.comb(total, row1)


def fisher_exact(s1, n1, s2, n2):
    """Two-sided Fisher exact p-value for the 2x2 table [[s1, n1-s1], [s2, n2-s2]].

    Sums every table with the same margins that is no more probable than the observed one
    (the convention scipy.stats.fisher_exact uses, with the same relative tolerance).
    """
    _counts(s1, n1)
    _counts(s2, n2)
    successes, total = s1 + s2, n1 + n2
    if total == 0 or successes in (0, total) or 0 in (n1, n2):
        return 1.0
    observed = _hypergeometric(s1, n1, successes, total)
    low, high = max(0, successes - n2), min(n1, successes)
    p = sum(
        prob
        for k in range(low, high + 1)
        if (prob := _hypergeometric(k, n1, successes, total)) <= observed * (1 + 1e-7)
    )
    return min(1.0, p)


def two_proportion_test(s1, n1, s2, n2, alpha=0.05):
    """Whether two success counts differ: pooled two-proportion z-test, two-sided, and Fisher's exact test.

    The verdict uses Fisher's exact test when any expected cell count is below 5 (the usual
    rule for when the normal approximation is not trustworthy), otherwise the z-test.
    """
    _counts(s1, n1)
    _counts(s2, n2)
    if n1 == 0 or n2 == 0:
        raise ValueError('both arms need at least one trial')
    if not 0 < alpha < 1:
        raise ValueError('alpha must be in (0, 1)')
    p1, p2 = s1 / n1, s2 / n2
    pooled = (s1 + s2) / (n1 + n2)
    se = math.sqrt(pooled * (1 - pooled) * (1 / n1 + 1 / n2))
    if se == 0:
        z, p_z = 0.0, 1.0
    else:
        z = (p2 - p1) / se
        p_z = 2 * (1 - _NORMAL.cdf(abs(z)))
    p_fisher = fisher_exact(s1, n1, s2, n2)
    expected = min(n * share for n in (n1, n2) for share in (pooled, 1 - pooled))
    method = 'fisher_exact' if expected < 5 else 'z_test'
    p = p_fisher if method == 'fisher_exact' else p_z
    return {
        'a': {'successes': s1, 'trials': n1, 'rate': p1, 'wilson95': wilson_interval(s1, n1)},
        'b': {'successes': s2, 'trials': n2, 'rate': p2, 'wilson95': wilson_interval(s2, n2)},
        'difference': p2 - p1,
        'z': z,
        'p_z_test': p_z,
        'p_fisher_exact': p_fisher,
        'min_expected_count': expected,
        'method': method,
        'p_value': p,
        'alpha': alpha,
        'differ': p < alpha,
    }


def trials_per_arm(p1, p2, alpha=0.05, power=0.8):
    """Trials per arm for a two-sided pooled two-proportion z-test to detect p1 vs p2.

    n = (z_{1-alpha/2} sqrt(2 pbar qbar) + z_{power} sqrt(p1 q1 + p2 q2))^2 / (p1 - p2)^2,
    rounded up, without continuity correction (the standard formula; e.g. Fleiss, Levin and
    Paik, Statistical Methods for Rates and Proportions, ch. 4). Near 0 or 1 or for small n it
    is optimistic: check the result by simulation when it matters.
    """
    if not (0 <= p1 <= 1 and 0 <= p2 <= 1) or p1 == p2:
        raise ValueError('p1 and p2 must be distinct proportions in [0, 1]')
    if not 0 < alpha < 1 or not 0 < power < 1:
        raise ValueError('alpha and power must be in (0, 1)')
    za, zb = _NORMAL.inv_cdf(1 - alpha / 2), _NORMAL.inv_cdf(power)
    pbar = (p1 + p2) / 2
    numerator = za * math.sqrt(2 * pbar * (1 - pbar)) + zb * math.sqrt(p1 * (1 - p1) + p2 * (1 - p2))
    # Guard the ceiling against binary noise in an exact integer result.
    return math.ceil(numerator**2 / (p1 - p2) ** 2 - 1e-9)


def percentile(values, q):
    """Linear-interpolation percentile (Hyndman and Fan type 7, numpy's default); None when empty."""
    if not 0 <= q <= 100:
        raise ValueError('q must be in [0, 100]')
    ordered = sorted(float(v) for v in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * q / 100
    low = math.floor(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)
