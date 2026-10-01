"""Scope the no-delay GNSS profile to the selected Gazebo receiver."""

import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'patches/px4/px4-rc.gzgnss'
SIM_ID = '11469068'


def initialize(tmp_path, mode='1', gazebo='1', gps='1', ids=('0', '0'), fail=False):
    commands = tmp_path / 'commands'
    environment = dict(
        PATH=os.defpath, PX4_GZ_GNSS_NO_DELAY=mode, TEST_GAZEBO=gazebo,
        TEST_GPS=gps, TEST_ID0=ids[0], TEST_ID1=ids[1],
        TEST_FAIL='1' if fail else '0', TEST_COMMANDS=str(commands), TEST_INIT=str(SCRIPT))
    shell = '''
param() {
  case "$1" in
    compare)
      case "$3" in
        SIM_GZ_EN) [ "$TEST_GAZEBO" = "$4" ] ;;
        SIM_GZ_EN_GPS) [ "$TEST_GPS" = "$4" ] ;;
        SENS_GPS0_ID) [ "$TEST_ID0" = "$4" ] ;;
        SENS_GPS1_ID) [ "$TEST_ID1" = "$4" ] ;;
        *) return 99 ;;
      esac ;;
    show) return 0 ;;
    set) echo "$2=$3" >> "$TEST_COMMANDS"; [ "$TEST_FAIL" = 0 ] ;;
    *) return 99 ;;
  esac
}
. "$TEST_INIT"
'''
    result = subprocess.run(['sh', '-c', shell], env=environment,
                            capture_output=True, text=True)
    written = commands.read_text().splitlines() if commands.exists() else []
    return result, written


@pytest.mark.parametrize('mode,gazebo,gps', [
    ('0', '1', '1'), ('1', '0', '1'), ('1', '1', '0'),
])
def test_other_sensor_profiles_are_unchanged(tmp_path, mode, gazebo, gps):
    result, written = initialize(tmp_path, mode, gazebo, gps)
    assert result.returncode == 0, result.stderr
    assert written == []


@pytest.mark.parametrize('ids,slot', [
    (('0', '0'), 0), ((SIM_ID, '1234'), 0), (('1234', SIM_ID), 1),
])
def test_zero_delay_only_for_matched_simulator_receiver(tmp_path, ids, slot):
    result, written = initialize(tmp_path, ids=ids)
    assert result.returncode == 0, result.stderr
    assert written == [f'SENS_GPS{slot}_DELAY=0']


def test_unknown_receiver_configuration_fails_before_setting_delay(tmp_path):
    result, written = initialize(tmp_path, ids=('1234', '0'))
    assert result.returncode == 1
    assert written == []


def test_parameter_write_failure_blocks_startup(tmp_path):
    result, _ = initialize(tmp_path, fail=True)
    assert result.returncode == 1


def test_hook_runs_before_simulator_and_is_registered():
    patch = (ROOT / 'patches/px4/gz_gnss_initialization.patch').read_text()
    assert patch.index('+\t. px4-rc.gzgnss') < patch.index('\t. px4-rc.simulator')
    assert '+\tpx4-rc.gzgnss\n' in patch
    launcher = (ROOT / 'scripts/start_px4_ros2.sh').read_text()
    assert 'cmp -s "${WS_ROOT}/patches/px4/px4-rc.gzgnss"' in launcher
    assert 'export PX4_GZ_GNSS_NO_DELAY=1' in launcher
