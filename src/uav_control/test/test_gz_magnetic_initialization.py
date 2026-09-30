"""Only the identified zero-bias Gazebo sensor gets ENU startup initialization."""

from pathlib import Path
import os
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'patches/px4/px4-rc.gzmag_enu'


def initialize(tmp_path, mode='1', gazebo='1', slot='0', device='197388'):
    commands = tmp_path / 'commands'
    environment = dict(PATH=os.defpath, PX4_GZ_MAGNETOMETER_ENU=mode, TEST_GAZEBO=gazebo,
                       TEST_SLOT=slot, TEST_DEVICE=device, TEST_COMMANDS=str(commands),
                       TEST_INIT=str(SCRIPT))
    # Parameter CLI is simulated; do not run PX4, modify BSON or start modules.
    shell = '''
param() {
  case "$1" in
    compare)
      if [ "$3" = SIM_GZ_EN ]; then [ "$TEST_GAZEBO" = "$4" ];
      else [ "$3" = "CAL_MAG${TEST_SLOT}_ID" ] && [ "$TEST_DEVICE" = "$4" ]; fi ;;
    show) echo "0.006228" ;;
    set) echo "$2=$3" >> "$TEST_COMMANDS" ;;
    *) return 99 ;;
  esac
}
. "$TEST_INIT"
'''
    subprocess.run(['sh', '-c', shell], env=environment, capture_output=True,
                   text=True, check=True)
    return commands.read_text().splitlines() if commands.exists() else []


@pytest.mark.parametrize('mode,gazebo,slot,device', [
    ('0', '1', '0', '197388'), ('1', '0', '0', '197388'), ('1', '1', '0', '1234'),
])
def test_other_modes_and_physical_devices_keep_calibration(tmp_path, mode, gazebo, slot, device):
    assert initialize(tmp_path, mode, gazebo, slot, device) == []


@pytest.mark.parametrize('slot', ['0', '2'])
def test_only_matching_slot_offsets_are_initialized(tmp_path, slot):
    assert initialize(tmp_path, slot=slot) == [
        f'CAL_MAG{slot}_{axis}OFF=0' for axis in 'XYZ']


def test_startup_hook_is_inside_simulator_gate_before_simulator_start():
    patch = (ROOT / 'patches/px4/gz_magnetic_initialization.patch').read_text()
    assert 'if ! replay tryapplyparams' in patch
    assert patch.index('+\t. px4-rc.gzmag_enu') < patch.index('\t. px4-rc.simulator')


def test_hook_is_registered_in_romfs_build_manifest():
    patch = (ROOT / 'patches/px4/gz_magnetic_initialization.patch').read_text()
    assert 'ROMFS/px4fmu_common/init.d-posix/CMakeLists.txt' in patch
    assert '+\tpx4-rc.gzmag_enu\n' in patch
