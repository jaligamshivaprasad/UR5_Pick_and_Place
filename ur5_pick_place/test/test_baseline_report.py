"""Signal-processing checks for the trajectory baseline report."""

import numpy as np
import pytest

from ur5_pick_place.baseline_report import (
    ARM_JOINTS, VectorSeries, _named_values, estimate_derivatives,
    interpolate_with_gap_limit, response_metrics,
)


def test_named_joint_mapping_ignores_input_order_and_extra_joints():
    names = ['extra', *reversed(ARM_JOINTS)]
    values = [99.0, *range(6)]
    mapped = _named_values(names, values)
    assert mapped.tolist() == [5, 4, 3, 2, 1, 0]


def test_alignment_rejects_extrapolation_and_large_timestamp_gaps():
    source_t = [0.0, 0.01, 0.02, 0.20, 0.21]
    values = np.column_stack([source_t, source_t])
    result, valid = interpolate_with_gap_limit(
        source_t, values, [-0.01, 0.015, 0.1, 0.205, 0.22])
    assert valid.tolist() == [False, True, False, True, False]
    assert result[1] == pytest.approx([0.015, 0.015])


@pytest.mark.parametrize('direction', [1.0, -1.0])
def test_response_metrics_handle_positive_and_negative_motion(direction):
    times = np.arange(0.0, 4.01, 0.01)
    reference = direction * np.minimum(times, 1.0)
    actual = direction * np.where(
        times < 1.0, times,
        1.0 + 0.1 * np.exp(-(times - 1.0) * 4.0) * np.cos((times - 1.0) * 8.0))
    velocity = np.gradient(actual, 0.01)
    jerk = np.gradient(np.gradient(velocity, 0.01), 0.01)
    result = response_metrics(
        times, reference, actual, velocity, jerk,
        endpoint_time=1.0, observation_end=4.0)
    assert result['Trajectory_Rise_Time_s'] == pytest.approx(0.8, abs=0.02)
    assert result['Overshoot_pct'] == pytest.approx(10.0, abs=0.1)
    assert result['Endpoint_Settling_Time_s'] == pytest.approx(0.41, abs=0.03)
    assert result['Rise_Status'] == result['Settling_Status'] == 'OK'


def test_short_endpoint_window_is_explicitly_not_a_settling_measurement():
    times = np.arange(0.0, 1.41, 0.01)
    reference = np.minimum(times, 1.0)
    actual = reference.copy()
    result = response_metrics(
        times, reference, actual, np.gradient(actual, 0.01), np.zeros_like(times),
        endpoint_time=1.0, observation_end=1.4)
    assert np.isnan(result['Endpoint_Settling_Time_s'])
    assert result['Settling_Status'] == 'InsufficientObservation'


def test_nonmonotonic_reference_does_not_claim_rise_or_overshoot():
    times = np.arange(0.0, 2.01, 0.01)
    reference = np.sin(times * np.pi / 2)
    result = response_metrics(
        times, reference, reference, np.gradient(reference, 0.01),
        np.zeros_like(times), endpoint_time=2.0, observation_end=2.0)
    assert result['Rise_Status'] == 'NotEligible'
    assert result['Overshoot_Status'] == 'NotEligible'
    assert np.isnan(result['Overshoot_pct'])


def test_savgol_derivatives_use_measured_velocity_and_omit_boundaries():
    times = np.arange(0.0, 2.01, 0.01)
    velocity = np.column_stack([(2.0 + joint) * times for joint in range(6)])
    series = VectorSeries(
        times=times.tolist(), record_times=times.tolist(),
        positions=[np.zeros(6) for _ in times],
        velocities=[row for row in velocity], accelerations=[])
    acceleration, jerk, quality = estimate_derivatives(series, times)
    interior = quality == 'OK'
    assert np.count_nonzero(interior) > 100
    assert acceleration[interior, 0] == pytest.approx(2.0, abs=1e-8)
    assert jerk[interior] == pytest.approx(0.0, abs=1e-7)
    assert quality[0] == 'InsufficientDerivativeWindow'
