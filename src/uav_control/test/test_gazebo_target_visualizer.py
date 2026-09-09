import xml.etree.ElementTree as ElementTree

import pytest

from uav_control.gazebo_target_visualizer import (
    ned_to_gazebo_enu,
    red_sphere_sdf,
)


def test_ned_to_gazebo_enu_swaps_horizontal_axes_and_flips_z():
    assert ned_to_gazebo_enu(10.0, 4.0, -3.0) == pytest.approx(
        (4.0, 10.0, 3.0)
    )


def test_red_sphere_is_visual_only_and_has_requested_size():
    model = ElementTree.fromstring(
        red_sphere_sdf('usv_target', 0.5)
    ).find('model')

    assert model is not None
    assert model.attrib['name'] == 'usv_target'
    assert model.findtext('static') == 'true'
    assert model.find('.//collision') is None
    radius = float(model.findtext('.//sphere/radius'))
    assert radius == pytest.approx(0.25)
    assert model.findtext('.//material/diffuse') == '1 0 0 1'


@pytest.mark.parametrize(
    'entity_name',
    ['', '1target', 'bad name', 'bad/name'],
)
def test_red_sphere_rejects_unsafe_entity_names(entity_name):
    with pytest.raises(ValueError):
        red_sphere_sdf(entity_name, 0.5)
