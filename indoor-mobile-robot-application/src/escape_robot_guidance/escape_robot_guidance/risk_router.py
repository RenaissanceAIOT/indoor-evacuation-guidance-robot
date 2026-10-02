"""Deterministic risk-aware graph routing used by the mission layer."""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from math import inf, isfinite
from typing import Iterable


@dataclass(frozen=True, slots=True)
class Edge:
    target: str
    distance_m: float
    hazard_risk: float = 0.0
    crowd_density: float = 0.0
    blocked: bool = False

    def cost(self, hazard_weight: float, crowd_weight: float) -> float:
        if self.blocked:
            return inf
        return self.distance_m + hazard_weight * self.hazard_risk + crowd_weight * self.crowd_density


@dataclass(frozen=True, slots=True)
class ExitScore:
    exit_id: str
    terminal_risk: float = 0.0
    crowd_density: float = 0.0
    enabled: bool = True


@dataclass(frozen=True, slots=True)
class RouteResult:
    exit_id: str
    path: tuple[str, ...]
    total_cost: float


class RiskGraph:
    def __init__(self) -> None:
        self._edges: dict[str, list[Edge]] = {}

    def add_edge(self, source: str, edge: Edge, *, bidirectional: bool = True) -> None:
        if any(not isfinite(v) or v < 0.0 for v in (
                edge.distance_m, edge.hazard_risk, edge.crowd_density)):
            raise ValueError("edge cost fields must be finite and nonnegative")
        self._edges.setdefault(source, []).append(edge)
        self._edges.setdefault(edge.target, [])
        if bidirectional:
            reverse = Edge(
                target=source,
                distance_m=edge.distance_m,
                hazard_risk=edge.hazard_risk,
                crowd_density=edge.crowd_density,
                blocked=edge.blocked,
            )
            self._edges[edge.target].append(reverse)

    def safest_exit(
        self,
        start: str,
        exits: Iterable[ExitScore],
        *,
        hazard_weight: float = 8.0,
        crowd_weight: float = 3.0,
    ) -> RouteResult | None:
        if start not in self._edges:
            raise KeyError(f"unknown start node: {start}")
        if any(not isfinite(v) or v < 0.0 for v in (hazard_weight, crowd_weight)):
            raise ValueError("weights must be finite and nonnegative")

        distances = {node: inf for node in self._edges}
        parents: dict[str, str] = {}
        distances[start] = 0.0
        queue: list[tuple[float, str]] = [(0.0, start)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance != distances[node]:
                continue
            for edge in self._edges[node]:
                candidate = distance + edge.cost(hazard_weight, crowd_weight)
                if candidate < distances[edge.target]:
                    distances[edge.target] = candidate
                    parents[edge.target] = node
                    heapq.heappush(queue, (candidate, edge.target))

        best: tuple[float, ExitScore] | None = None
        for exit_score in exits:
            if any(not isfinite(v) or v < 0.0 for v in (
                    exit_score.terminal_risk, exit_score.crowd_density)):
                raise ValueError("exit cost fields must be finite and nonnegative")
            if not exit_score.enabled or exit_score.exit_id not in distances:
                continue
            total = (
                distances[exit_score.exit_id]
                + hazard_weight * exit_score.terminal_risk
                + crowd_weight * exit_score.crowd_density
            )
            candidate = (total, exit_score)
            if best is None or (candidate[0], exit_score.exit_id) < (best[0], best[1].exit_id):
                best = candidate
        if best is None or best[0] == inf:
            return None

        total, selected = best
        path = [selected.exit_id]
        while path[-1] != start:
            path.append(parents[path[-1]])
        path.reverse()
        return RouteResult(selected.exit_id, tuple(path), total)
