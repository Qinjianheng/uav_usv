"""Simulation magnetic vectors must retain physical heading at every pose."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[3]


def test_native_adapter_uses_sensor_flu_vector_for_enu_world(tmp_path):
    header = ROOT / 'patches/px4/GzMagneticField.hpp'
    assert header.is_file(), 'ENU/FLU magnetometer adapter is missing'
    source = tmp_path / 'mag.cpp'
    source.write_text(
        '#include "GzMagneticField.hpp"\n#include <iostream>\n'
        'int main(){float x,y,z;int enu;while(std::cin>>x>>y>>z>>enu){'
        'auto m=px4_gz::magnetometer_frd(x,y,z,enu);'
        'std::cout<<m[0]<<" "<<m[1]<<" "<<m[2]<<"\\n";}}\n')
    binary = tmp_path / 'mag'
    subprocess.run(['c++', '-std=c++14', '-I', str(header.parent),
                    str(source), '-o', str(binary)], check=True)
    # Independent ENU field and rotated FLU sensors, including tilt/yaw.
    world = np.array([.012, .22, -.43])
    inputs, expected = [], []
    for angles in [(0, 0, 0), (0, 0, 90), (0, 0, -45), (12, -8, 123)]:
        body_flu = Rotation.from_euler('xyz', angles, degrees=True).inv().apply(world)
        inputs.append(' '.join(map(str, (*body_flu, 1))))
        expected.append(body_flu * [1., -1., -1.])
    # Existing worlds remain in explicitly unselected legacy mode.
    inputs.append('1 2 3 0')
    expected.append([-2., -1., 3.])
    result = subprocess.run([str(binary)], input='\n'.join(inputs)+'\n',
                            text=True, capture_output=True, check=True)
    actual = np.array([[float(v) for v in line.split()]
                       for line in result.stdout.splitlines()])
    np.testing.assert_allclose(actual, expected, atol=1e-6)


def test_server_config_selects_enu_gauss_without_removing_other_systems(tmp_path):
    original = tmp_path / 'server.config'
    original.write_text('<server_config><plugins>'
                        '<plugin name="gz::sim::systems::Physics" filename="physics"/>'
                        '<plugin name="gz::sim::systems::Magnetometer" filename="mag"/>'
                        '<plugin name="gz::sim::systems::Sensors" filename="sensors">'
                        '<render_engine>ogre2</render_engine></plugin>'
                        '</plugins></server_config>')
    output = tmp_path / 'enu.config'
    script = ROOT / 'scripts/prepare_gz_magnetometer_config.py'
    subprocess.run([sys.executable, str(script), '--source', str(original),
                    '--output', str(output)], check=True)
    plugins = ET.parse(output).getroot().findall('plugins/plugin')
    assert [p.attrib['filename'] for p in plugins] == ['physics', 'mag', 'sensors']
    assert plugins[1].findtext('use_earth_frame_ned') == 'false'
    assert plugins[1].findtext('use_units_gauss') == 'true'
    assert plugins[2].findtext('render_engine') == 'ogre2'
