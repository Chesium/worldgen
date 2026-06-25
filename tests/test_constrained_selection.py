from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config, load_config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.partition import Partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    RoomGraphSelectionError,
    RoomSelection,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
    validate_passage_constraints,
)
from random_gazebo_world import pipeline


def _sample_config(**overrides: float | int) -> Config:
    values = {
        "world_width": 15.0,
        "world_height": 15.0,
        "min_cell_size": 2.0,
        "max_cell_size": 6.0,
        "min_room_count": 3,
        "max_room_count": 8,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.8,
        "gate_width_max": 1.2,
        "passage_width_min": 0.8,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.5,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _grid_3x3() -> Partition:
    cells = tuple(
        Cell.from_origin_size(
            row * 3 + col,
            col * 5.0,
            row * 5.0,
            5.0,
            5.0,
        )
        for row in range(3)
        for col in range(3)
    )
    return Partition(cells=cells, world_width=15.0, world_height=15.0)


def _build_structure(room_ids: set[int], config: Config):
    partition = _grid_3x3()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition, room_cell_ids=frozenset(room_ids)
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    return adjacency, candidates


# Ring of eight rooms around an unused center cell. Naive selection can route
# several passages through the center (a junction); the constrained selection
# must keep the center within the configured edge budget.
_RING_ROOMS = {0, 1, 2, 3, 5, 6, 7, 8}


def test_constrained_selection_respects_passage_limits() -> None:
    config = _sample_config(
        max_open_edges_per_passage=2,
        max_openings_per_passage_edge=1,
    )
    adjacency, candidates = _build_structure(_RING_ROOMS, config)

    selected = select_room_graph(
        candidates, adjacency, config, create_seeded_rng(7)
    )
    layout = apply_connections(selected, adjacency)

    # Should not raise: limits are satisfied by construction.
    validate_passage_constraints(layout, config)


def test_constrained_selection_is_deterministic_for_a_seed() -> None:
    config = _sample_config(
        max_open_edges_per_passage=2,
        max_openings_per_passage_edge=1,
    )
    adjacency, candidates = _build_structure(_RING_ROOMS, config)

    first = select_room_graph(candidates, adjacency, config, create_seeded_rng(123))
    second = select_room_graph(candidates, adjacency, config, create_seeded_rng(123))
    assert first == second


def test_constrained_selection_holds_across_many_seeds() -> None:
    config = _sample_config(
        max_open_edges_per_passage=2,
        max_openings_per_passage_edge=1,
    )
    adjacency, candidates = _build_structure(_RING_ROOMS, config)

    for seed in range(40):
        selected = select_room_graph(
            candidates, adjacency, config, create_seeded_rng(seed)
        )
        layout = apply_connections(selected, adjacency)
        validate_passage_constraints(layout, config)


def test_inner_retry_recovers_from_selection_dead_end(monkeypatch) -> None:
    config = load_config(Path("configs/default.yaml"))
    real_select = pipeline.select_room_graph
    calls = {"count": 0}

    def flaky_select(candidates, adjacency, cfg, rng):
        calls["count"] += 1
        if calls["count"] == 1:
            raise RoomGraphSelectionError("forced dead end on first selection")
        return real_select(candidates, adjacency, cfg, rng)

    monkeypatch.setattr(pipeline, "select_room_graph", flaky_select)

    world = pipeline.generate_valid_world(config)

    assert calls["count"] >= 2
    validate_passage_constraints(world.applied_layout, config)


def test_debug_retries_emits_periodic_summary(monkeypatch, capsys) -> None:
    config = load_config(Path("configs/default.yaml"))
    real_select = pipeline.select_room_graph
    calls = {"count": 0}

    def flaky_select(candidates, adjacency, cfg, rng):
        calls["count"] += 1
        if calls["count"] == 1:
            raise RoomGraphSelectionError("forced dead end on first selection")
        return real_select(candidates, adjacency, cfg, rng)

    monkeypatch.setattr(pipeline, "select_room_graph", flaky_select)
    world = pipeline.generate_valid_world(
        config,
        debug_retries=True,
        debug_retry_summary_interval=1,
    )

    stderr = capsys.readouterr().err
    assert calls["count"] >= 2
    assert world.attempt >= 0
    assert "[debug-retries] retry summary;" in stderr
    assert "stages=[selection:1]" in stderr
    assert "selected_rooms=" in stderr
    assert "candidates=" in stderr
    assert "last_error=RoomGraphSelectionError: forced dead end on first selection" in stderr
