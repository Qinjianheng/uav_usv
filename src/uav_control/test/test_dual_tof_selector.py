from uav_control.perception.dual_tof_selector import (
    HystereticCameraSelector,
)


def test_selector_stays_front_through_short_dropout():
    selector = HystereticCameraSelector(
        loss_frames=3,
        front_recovery_frames=5,
    )

    assert selector.update(True, False)[0] == 'front'
    assert selector.update(False, True)[0] == 'front'
    assert selector.update(False, True)[0] == 'front'
    assert selector.update(True, True)[0] == 'front'


def test_selector_switches_down_after_sustained_front_loss():
    selector = HystereticCameraSelector(
        loss_frames=3,
        front_recovery_frames=5,
    )

    states = [
        selector.update(False, True)[0]
        for _ in range(3)
    ]

    assert states == ['front', 'front', 'down']


def test_selector_requires_stable_front_recovery_before_switching_back():
    selector = HystereticCameraSelector(
        loss_frames=1,
        front_recovery_frames=3,
    )
    assert selector.update(False, True)[0] == 'down'

    states = [
        selector.update(True, True)[0]
        for _ in range(3)
    ]

    assert states == ['down', 'down', 'front']
