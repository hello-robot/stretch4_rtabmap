import os

import pytest
from stretch4_rtabmap.localization.rtabmap_params import mode_params, same

CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', 'config')


def test_shipped_files_swap_the_icp_settings():
    mapping, localization = mode_params(os.path.join(CONFIG, 'rtabmap_params_mapping.yaml'),
                                        os.path.join(CONFIG, 'rtabmap_params_localization.yaml'))
    assert mapping['Icp/MaxCorrespondenceDistance'] == '0.1'
    assert localization['Icp/MaxCorrespondenceDistance'] == '0.3'
    assert set(mapping) == set(localization) == {
        'Icp/MaxCorrespondenceDistance', 'Icp/MaxTranslation',
        'RGBD/LocalRadius'}


def test_a_key_in_one_file_only_is_refused(tmp_path):
    (tmp_path / 'm.yaml').write_text('rtabmap_parameters:\n  Icp/Iterations: 30\n')
    (tmp_path / 'l.yaml').write_text('inherit: m.yaml\nrtabmap_parameters:\n  Icp/Foo: 1\n')
    with pytest.raises(RuntimeError, match='Icp/Foo'):
        mode_params(str(tmp_path / 'm.yaml'), str(tmp_path / 'l.yaml'))


def test_same_compares_numbers_by_value():
    assert same('0.10', '0.1') and same('True', 'true') and not same('0.1', '0.3')
