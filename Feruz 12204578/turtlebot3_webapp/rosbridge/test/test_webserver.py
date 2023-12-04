# Author: Feruz 12204578

import pytest
from click.testing import CliRunner
from webserver import run_webserver

@pytest.fixture
def runner():
    return CliRunner()

def test_run_webserver_defaults(runner):
    # Test the default behavior of run_webserver command
    result = runner.invoke(run_webserver, [])
    assert result.exit_code == 0
    assert "Initialized" in result.output

def test_run_webserver_custom_args(runner):
    # Test the run_webserver command with custom arguments
    args = ["--name", "custom_name", "--port", "8080", "--webpath", "custom_path", "--cached", "false", "--start_port", "9000", "--end_port", "10000"]
    result = runner.invoke(run_webserver, args)
    assert result.exit_code == 0
    assert "Initialized" in result.output
