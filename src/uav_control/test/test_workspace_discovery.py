"""Evidence archives must never be discovered as buildable packages."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_default_colcon_discovery_only_finds_source_packages():
    result = subprocess.run(
        ['colcon', 'list', '--base-paths', str(ROOT), '--paths-only'],
        cwd=ROOT, capture_output=True, text=True, check=True)
    packages = [Path(line).resolve() for line in result.stdout.splitlines() if line]
    assert packages
    assert all(path.is_relative_to(ROOT / 'src') for path in packages), packages
