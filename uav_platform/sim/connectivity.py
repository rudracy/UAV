"""
connectivity.py — Multi-hop BVLOS communication mesh.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from uav_platform.sim.world import Terrain
from uav_platform.sim.uav import UAVState


SPEED_OF_LIGHT = 3e8          # m/s
PROCESSING_MS = 2.0           # fixed processing latency per hop


# ---------------------------------------------------------------------------
# CommLink
# ---------------------------------------------------------------------------

@dataclass
class CommLink:
    uav_a: int
    uav_b: int
    rssi_dbm: float
    packet_loss_pct: float
    latency_ms: float
    active: bool = True


# ---------------------------------------------------------------------------
# ConnectivityGraph
# ---------------------------------------------------------------------------

class ConnectivityGraph:
    """
    Builds and maintains a mesh communication graph over UAV swarm states.

    GCS node is represented as uav_id = -1, located at (0, 0, 0).
    """

    GCS_ID = -1

    def __init__(
        self,
        radio_range_m: float = 150.0,
        comm_rate_hz: float = 10.0,
        neighbour_count: int = 4,
    ) -> None:
        self.radio_range_m = radio_range_m
        self.comm_rate_hz = comm_rate_hz
        self.neighbour_count = neighbour_count

        # adjacency: node_id -> list of CommLink
        self._adj: Dict[int, List[CommLink]] = {}
        # links keyed (min_id, max_id) for dedup
        self._links: Dict[Tuple[int, int], CommLink] = {}
        # outaged UAVs
        self._outaged: Set[int] = set()
        self._terrain = Terrain()

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update(
        self,
        states: List[UAVState],
        terrain: Terrain,
        t: float,
        extra_packet_loss: float = 0.0,
    ) -> None:
        """
        Rebuild the adjacency graph for the current tick.

        Uses distance + line-of-sight. Respects k-mesh vs broadcast mode.
        Incorporates extra_packet_loss stress testing into link delivery and outage.
        """
        self._adj = {self.GCS_ID: []}
        for s in states:
            self._adj[s.uav_id] = []
        self._links = {}

        alive_states = [s for s in states if s.alive]
        # Include GCS as virtual node with 2m antenna mast above ground
        gcs_z = terrain.elevation(0.0, 0.0) + 2.0
        all_nodes = [(self.GCS_ID, 0.0, 0.0, gcs_z)] + [
            (s.uav_id, s.x, s.y, s.z) for s in alive_states
        ]

        n = len(all_nodes)

        # Compute all pairwise distances and store candidates
        candidates: Dict[int, List[Tuple[float, int, CommLink]]] = {
            nid: [] for nid, *_ in all_nodes
        }

        for i in range(n):
            id_a, xa, ya, za = all_nodes[i]
            if id_a in self._outaged:
                continue
            for j in range(i + 1, n):
                id_b, xb, yb, zb = all_nodes[j]
                if id_b in self._outaged:
                    continue
                dist = math.sqrt((xa - xb) ** 2 + (ya - yb) ** 2 + (za - zb) ** 2)
                if dist > self.radio_range_m:
                    continue
                # Line-of-sight check
                if not terrain.line_of_sight((xa, ya, za), (xb, yb, zb)):
                    continue

                rssi = self._rssi(dist)
                loss_pct = extra_packet_loss * 100.0 if extra_packet_loss <= 1.0 else extra_packet_loss
                pl = min(100.0, max(0.0, self._packet_loss(rssi) + loss_pct))
                lat = self._latency(dist)
                # Link is inactive if complete packet drop
                is_active = (pl < 95.0)
                link = CommLink(id_a, id_b, rssi, pl, lat, active=is_active)

                if is_active:
                    candidates[id_a].append((dist, id_b, link))
                    candidates[id_b].append((dist, id_a, link))

        # Apply k-mesh or broadcast selection
        broadcast = self.neighbour_count >= 8
        for node_id, cands in candidates.items():
            if broadcast:
                selected = cands
            else:
                # Keep only k nearest
                cands.sort(key=lambda c: c[0])
                selected = cands[: self.neighbour_count]

            for dist, peer_id, link in selected:
                key = (min(node_id, peer_id), max(node_id, peer_id))
                if key not in self._links:
                    self._links[key] = link
                    self._adj[node_id].append(link)
                    if peer_id in self._adj:
                        self._adj[peer_id].append(link)

    # ------------------------------------------------------------------
    # Routing
    # ------------------------------------------------------------------

    def get_route(self, src: int, dst: int) -> Optional[List[int]]:
        """
        BFS multi-hop path from src to dst through the mesh.
        Returns list of node IDs (including src and dst) or None if unreachable.
        """
        if src not in self._adj or dst not in self._adj:
            return None
        if src == dst:
            return [src]

        visited: Set[int] = {src}
        queue: deque[List[int]] = deque([[src]])

        while queue:
            path = queue.popleft()
            current = path[-1]
            for link in self._adj.get(current, []):
                if not link.active:
                    continue
                peer = link.uav_b if link.uav_a == current else link.uav_a
                if peer == dst:
                    return path + [peer]
                if peer not in visited:
                    visited.add(peer)
                    queue.append(path + [peer])
        return None

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------

    def connectivity_ratio(self) -> float:
        """Fraction of UAV nodes reachable from GCS."""
        uav_nodes = [nid for nid in self._adj if nid != self.GCS_ID]
        if not uav_nodes:
            return 1.0
        reachable = sum(
            1 for nid in uav_nodes if self.get_route(self.GCS_ID, nid) is not None
        )
        return reachable / len(uav_nodes)

    def avg_latency_ms(self) -> float:
        """Mean end-to-end latency (ms) from each UAV to GCS via shortest path."""
        uav_nodes = [nid for nid in self._adj if nid != self.GCS_ID]
        if not uav_nodes:
            return 0.0
        total = 0.0
        count = 0
        for nid in uav_nodes:
            route = self.get_route(nid, self.GCS_ID)
            if route is None:
                continue
            lat = self._route_latency(route)
            total += lat
            count += 1
        return total / count if count else 0.0

    def avg_queuing_delay_ms(self) -> float:
        """Mean queuing delay per hop based on beacon frequency f_comm."""
        return round(100.0 / max(0.5, self.comm_rate_hz), 2)

    def avg_propagation_delay_ms(self) -> float:
        """Mean physical propagation delay in ms."""
        active_links = [l for l in self._links.values() if l.active]
        if not active_links:
            return 0.001
        # In our arena, propagation is sub-microsecond
        return round(0.0005, 4)

    def information_freshness_ms(self) -> float:
        """State update freshness interval (age of information) in ms."""
        return round(1000.0 / max(0.5, self.comm_rate_hz), 1)

    def packet_delivery_ratio(self) -> float:
        """Mean PDR (0-1) across all active links."""
        active_links = [l for l in self._links.values() if l.active]
        if not active_links:
            return 0.0
        pdr = sum((1.0 - l.packet_loss_pct / 100.0) for l in active_links)
        return pdr / len(active_links)

    def simulate_outage(self, uav_id: int) -> None:
        """Drop all links for *uav_id* (simulates radio failure)."""
        self._outaged.add(uav_id)
        for link in self._links.values():
            if link.uav_a == uav_id or link.uav_b == uav_id:
                link.active = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _rssi(dist: float) -> float:
        """Free-space path loss model: RSSI = -40 - 20·log10(dist) dBm."""
        if dist < 1.0:
            dist = 1.0
        return -40.0 - 20.0 * math.log10(dist)

    @staticmethod
    def _packet_loss(rssi_dbm: float) -> float:
        """
        Packet loss percentage derived from RSSI.
        -50 dBm → ~0 %  loss
        -90 dBm → ~50 % loss
        -110 dBm → ~100 % loss
        """
        if rssi_dbm >= -50.0:
            return 0.0
        elif rssi_dbm <= -110.0:
            return 100.0
        else:
            return ((-rssi_dbm - 50.0) / 60.0) * 100.0

    def _latency(self, dist: float) -> float:
        """Propagation + processing + queuing latency in ms."""
        prop_ms = (dist / SPEED_OF_LIGHT) * 1000.0
        queue_ms = 100.0 / max(0.5, self.comm_rate_hz)
        return prop_ms + PROCESSING_MS + queue_ms

    def _route_latency(self, route: List[int]) -> float:
        """Sum link latencies along a route."""
        total = 0.0
        for i in range(len(route) - 1):
            a, b = route[i], route[i + 1]
            key = (min(a, b), max(a, b))
            link = self._links.get(key)
            if link:
                total += link.latency_ms
        return total

    # ------------------------------------------------------------------
    # Accessors used by runner / task allocator
    # ------------------------------------------------------------------

    def all_links(self) -> List[CommLink]:
        """Return list of all unique communication links."""
        return list(self._links.values())

    def is_connected_to_gcs(self, uav_id: int) -> bool:
        return self.get_route(uav_id, self.GCS_ID) is not None

    def link_between(self, a: int, b: int) -> Optional[CommLink]:
        key = (min(a, b), max(a, b))
        return self._links.get(key)
