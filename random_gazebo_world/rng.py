from __future__ import annotations

import random


def create_seeded_rng(seed: int) -> random.Random:
    return random.Random(seed)
