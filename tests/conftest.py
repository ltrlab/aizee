"""The gate's parallel phase (scripts/dev.py run_tests: pytest-xdist with --dist loadgroup): each test file stays on one
worker, in order, as in a single process (some tests need the one before them, others share a module's fixtures),
except SPLIT's files, whose tests are independent and long enough to spread across the workers."""

import pytest

# Files whose tests may run on any worker, in any order: each sets up its own receiver and simulation. Add a file only
# after checking it has no module-scoped fixture and no test that relies on another.
SPLIT = {'tests/contract/test_rehearsal.py'}


@pytest.hookimpl(tryfirst=True)  # Before xdist's own hook, which turns each test's group mark into its group.
def pytest_collection_modifyitems(config, items):
    # The workers collect, and see the controller's --dist loadgroup as option.loadgroup: their option.dist is 'no'.
    # Read from option.dist, the groups never took, and a file's dependent tests split across workers (2026-10-30).
    if not getattr(config.option, 'loadgroup', False):
        return
    for item in items:
        path = item.nodeid.split('::')[0]
        if path not in SPLIT:
            item.add_marker(pytest.mark.xdist_group(path))
