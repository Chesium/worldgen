from __future__ import annotations

from pathlib import Path

from random_gazebo_world import cli
from random_gazebo_world.config import load_config


def test_generate_parser_accepts_debug_retry_flags() -> None:
    parser = cli.build_parser()
    args = parser.parse_args(
        [
            "generate",
            "--config",
            "configs/default.yaml",
            "--out",
            "outputs/test_world",
            "--debug-retries",
            "--debug-retries-summary-interval",
            "7",
        ]
    )

    assert args.debug_retries is True
    assert args.debug_retries_summary_interval == 7


def test_main_passes_debug_retry_options(monkeypatch, tmp_path: Path) -> None:
    captured: dict[str, object] = {}
    config = load_config(Path("configs/default.yaml"))

    def fake_load_config(path: Path):
        captured["config_path"] = path
        return config

    def fake_generate_world(
        cfg,
        out_dir: Path,
        debug_retries: bool,
        debug_retries_summary_interval: int,
    ) -> None:
        captured["config"] = cfg
        captured["out_dir"] = out_dir
        captured["debug_retries"] = debug_retries
        captured["debug_retries_summary_interval"] = debug_retries_summary_interval

    monkeypatch.setattr(cli, "load_config", fake_load_config)
    monkeypatch.setattr(cli, "generate_world", fake_generate_world)

    exit_code = cli.main(
        [
            "generate",
            "--config",
            "configs/default.yaml",
            "--out",
            str(tmp_path / "world"),
            "--debug-retries",
            "--debug-retries-summary-interval",
            "3",
        ]
    )

    assert exit_code == 0
    assert captured["config_path"] == Path("configs/default.yaml")
    assert captured["config"] == config
    assert captured["out_dir"] == tmp_path / "world"
    assert captured["debug_retries"] is True
    assert captured["debug_retries_summary_interval"] == 3
