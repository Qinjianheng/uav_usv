import importlib

import pytest


def test_rate_meter_reports_recent_callback_frequency():
    """Catch frame-rate diagnostics counting processed timers as raw frames."""
    runtime_performance = importlib.import_module(
        'uav_control.common.runtime_performance'
    )
    meter = runtime_performance.RateMeter(window_seconds=1.0)
    meter.observe(10.00)
    meter.observe(10.05)
    meter.observe(10.10)

    assert meter.rate(10.10) == pytest.approx(20.0)


def test_rate_meter_drops_samples_outside_window():
    runtime_performance = importlib.import_module(
        'uav_control.common.runtime_performance'
    )
    meter = runtime_performance.RateMeter(window_seconds=0.2)
    meter.observe(1.0)
    meter.observe(1.1)
    meter.observe(1.4)

    assert meter.rate(1.4) == 0.0
