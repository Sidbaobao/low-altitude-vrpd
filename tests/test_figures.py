"""The analysis-figure script must run on the stored results of the full run."""
import os
import sys

import pytest

pytest.importorskip("matplotlib")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

HAVE_RESULTS = os.path.exists(os.path.join(ROOT, "results", "routes.json"))


@pytest.mark.skipif(not HAVE_RESULTS, reason="results/routes.json not present")
def test_make_figures_from_stored_results(tmp_path):
    import make_figures

    # load() also re-evaluates every stored plan and asserts it equals the stored objective
    make_figures.main(["--results", os.path.join(ROOT, "results"), "--out", str(tmp_path),
                       "--skip-ga", "--skip-robustness"])
    for name in ("fig_sensitivity", "fig_tradeoff"):
        assert (tmp_path / f"{name}.png").stat().st_size > 10_000
        assert (tmp_path / f"{name}.pdf").stat().st_size > 1_000
