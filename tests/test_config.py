import pytest

from humkit.config import load_config
from humkit.error import ConfigError, HumkitError


def test_load_config_reads_a_mapping(tmp_path):
    path = tmp_path / "pd.yaml"
    path.write_text("type: pd\nkp: 50.0\nkd: 2.0\n")
    assert load_config(path) == {"type": "pd", "kp": 50.0, "kd": 2.0}


@pytest.mark.parametrize("text", ["- pd\n- gravity_comp\n", "just a string\n", ""])
def test_load_config_rejects_non_mapping(tmp_path, text):
    path = tmp_path / "bad.yaml"
    path.write_text(text)
    with pytest.raises(ConfigError):
        load_config(path)


def test_config_error_is_catchable_as_humkit_and_value_error():
    assert issubclass(ConfigError, HumkitError)
    assert issubclass(ConfigError, ValueError)
