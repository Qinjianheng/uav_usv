import importlib


def test_pause_retries_once_and_reports_terminal_action_status():
    """Catch terminal pause being attempted only once through moving_target."""
    gazebo_terminal = importlib.import_module(
        'uav_control.evaluation.gazebo_terminal'
    )
    responses = iter((False, True))
    attempts = []

    pauser = gazebo_terminal.GazeboTerminalPauser(
        pause_request=lambda: attempts.append(True) or next(responses),
        maximum_attempts=2,
        retry_delay=0.0,
        sleep=lambda _delay: None,
    )
    result = pauser.pause('SEA_CONTACT')

    assert result.terminal_event == 'SEA_CONTACT'
    assert result.gazebo_pause_requested
    assert result.gazebo_pause_succeeded
    assert result.attempts == 2
    assert len(attempts) == 2
