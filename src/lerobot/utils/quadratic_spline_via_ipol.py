#!/usr/bin/env python3
"""
Minimal quadratic-spline joint motion generator 
"""

import math
import warnings
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# ---------- Small helpers ----------
NUM_TOL = 1e-9
S_STEPS = 200              # base sampling density per "unit s" between via points

@dataclass
class Limits:
    """Per-joint limits (constant along the path)."""
    max_vel: np.ndarray  # shape (J,)
    max_acc: np.ndarray  # shape (J,)


@dataclass
class Via:
    """Via point + allowed max deviation at that via (per joint or scalar)."""
    q: np.ndarray         # shape (J,)
    max_dev: np.ndarray   # shape (J,) or scalar broadcastable


@dataclass
class Piece:
    """One quadratic piece q(s) = a + b*s_loc + c*s_loc^2 on [s_left, s_right]."""
    a: np.ndarray         # shape (J,)
    b: np.ndarray
    c: np.ndarray
    s_left: float
    s_right: float


class QuadraticSplineInterpolator:
    """
    Minimal quadratic spline + time-parameterization:
      - generate quadratic pieces with deviation control
      - compute ds limits from joint vel/acc
      - forward/backward pass for feasible ds(s)
      - resample in time
    """

    def __init__(self, via: List[Via], limits: Limits):
        assert len(via) >= 2, "Need at least two via points"
        self.J = via[0].q.size
        self.via = via
        self.lim = limits

        # Working containers
        self.pieces: List[Piece] = []
        self.s_borders: List[float] = []  # monotone borders for pieces
        self.samples_s = None
        self.ds_max = None
        self.ds = None                    # feasible ds after fwd/bwd
        self.q_samp = None                # (N, J)
        self.dqds_samp = None             # (N, J)
        self.ddqdds_samp = None           # (N, J)

    # ---- Spline building ----
    @staticmethod
    def _solve_piece(q1, q2, q3, ds) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Solve coefficients for one piece that "peaks" at middle via with offset ds in s.
        """
        b = (q2 - q1)
        a = q1 + (1.0 - ds) * b
        if ds > 0.0:
            c = (q2 + ds * (q3 - q2) - a - 2.0 * b * ds) / (4.0 * ds * ds)
        else:
            c = np.zeros_like(a)
        return a, b, c

    def _build_pieces(self):
        """
        Build 2 pieces per via interval [i,i+1]: (quadratic arc) + (straight connector),
        adapting ds (offset) to meet per-via max deviation constraints.
        """
        # --- special case: exactly two vias -> single straight segment 0..1 (keep if you added it) ---
        if len(self.via) == 2:
            q0 = self.via[0].q
            q1 = self.via[1].q
            self.pieces = [Piece(a=q0, b=(q1 - q0), c=np.zeros_like(q0), s_left=0.0, s_right=1.0)]
            self.s_borders = [0.0, 1.0]
            return

        # Duplicate first and last via
        ext = [self.via[0]] + self.via + [self.via[-1]]
        pieces: List[Piece] = []
        for i in range(len(self.via)-1):  # i: via interval i..i+1
            # choose ds (offset) starting at 0.5, reduce if deviation too large
            min_offset = 0.5

            q1 = ext[i].q
            q2 = ext[i+1].q
            q3 = ext[i+2].q
            max_dev = np.asarray(ext[i+1].max_dev)

            # compute candidate with ds = 0.5 and measure deviation at center s_loc = 0.5
            a0, b0, c0 = self._solve_piece(q1, q2, q3, 0.5)
            dev = q2 - (a0 + 0.5*b0 + 0.25*c0)
            # Any joint exceeding its allowed deviation? shrink ds accordingly
            if np.any(np.abs(dev) > max_dev):
                denom = (-q1 + 2*q2 - q3).copy()
                small = np.abs(denom) < NUM_TOL
                # force ±NUM_TOL depending on sign; if denom==0, pick +NUM_TOL
                denom[small] = np.where(denom[small] >= 0.0, NUM_TOL, -NUM_TOL)
                # per-joint offset required
                offset_j = np.abs(4.0 * max_dev / denom)
                min_offset = min(min_offset, float(np.min(offset_j)))

            # lower bound for ds so we still have resolution to sample the piece
            min_offset = max(min_offset, 2.0 / S_STEPS)

            # borders: place two borders around via i+1 at offset
            left = (i+1) - min_offset
            right = (i+1) + min_offset

            # final coefficients for the quadratic arc on [left, right]
            a, b, c = self._solve_piece(q1, q2, q3, min_offset)
            pieces.append(Piece(a=a, b=b, c=c, s_left=left, s_right=right))
            
            # connector only if there will be a next arc
            if i < len(self.via) - 2:
                s_loc = min_offset
                q_right = a + b*s_loc + c*(s_loc**2)
                dqds_right = b + 2*c*s_loc
                pieces.append(Piece(
                    a=q_right, b=dqds_right, c=np.zeros_like(q_right),
                    s_left=right, s_right=np.nan
                ))

        # close connector intervals to next arc's left border
        for k in range(1, len(pieces) - 1, 2):
            pieces[k].s_right = pieces[k+1].s_left

        self.pieces = pieces
        # compute borders from pieces only
        self.s_borders = sorted({p.s_left for p in pieces} | {p.s_right for p in pieces})

    # ---- Sampling & limits ----
    def _eval_piece(self, piece: Piece, s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return q, dq/ds, d2q/ds2 at absolute s inside this piece."""
        s_loc = s - piece.s_left
        q = piece.a + piece.b * s_loc + piece.c * s_loc * s_loc
        dqds = piece.b + 2.0 * piece.c * s_loc
        ddqdds = 2.0 * piece.c
        return q, dqds, ddqdds

    def _piece_at(self, s: float) -> Piece:
        for p in self.pieces:
            if p.s_left - NUM_TOL <= s <= p.s_right + NUM_TOL:
                return p
        # clamp to edges if slightly outside due to numeric
        return self.pieces[-1] if s > self.pieces[-1].s_right else self.pieces[0]

    def _build_samples(self):
        """Create s grid including all borders + uniform interior points; compute q,dqds,ddqdds and ds_max."""
        # s runs roughly from 1 - offset_min to (N-1) + offset_min
        s_min = self.s_borders[0]
        s_max = self.s_borders[-1]
        # base step: adapt to local piece lengths
        total_units = max(1.0, s_max - s_min)
        N_uniform = max(2 * S_STEPS, int(math.ceil(total_units * S_STEPS)))
        s_uniform = np.linspace(s_min, s_max, N_uniform)

        # include all borders explicitly to be safe
        s_all = np.unique(np.concatenate([s_uniform, np.array(self.s_borders)]))
        s_all.sort()

        N = s_all.size
        J = self.J
        q = np.zeros((N, J))
        dqds = np.zeros((N, J))
        ddqdds = np.zeros((N, J))
        ds_max = np.full(N, np.inf)

        for i, s in enumerate(s_all):
            pc = self._piece_at(s)
            qi, dqi, ddi = self._eval_piece(pc, s)
            q[i], dqds[i], ddqdds[i] = qi, dqi, ddi

            # velocity bound: |dq/dt| = |dq/ds| * ds <= max_vel  ->  ds <= max_vel / |dq/ds|
            with np.errstate(divide='ignore', invalid='ignore'):
                v_bound = np.where(np.abs(dqi) > NUM_TOL, self.lim.max_vel / np.abs(dqi), np.inf)
            ds_v = np.min(v_bound)

            # acceleration bound: |d2q/dt2| = | d2q/ds2 * ds^2 + dq/ds * dds | <= max_acc
            # Using a conservative bound: require |d2q/ds2| * ds^2 <= max_acc  -> ds <= sqrt(max_acc / |ddqdds|)
            # (this ignores the dds term, which we’ll enforce during fwd/bwd integration)
            with np.errstate(divide='ignore', invalid='ignore'):
                a_bound = np.where(np.abs(ddi) > NUM_TOL, np.sqrt(self.lim.max_acc / np.abs(ddi)), np.inf)
            ds_a = float(np.min(a_bound))
            val = min(ds_v, ds_a)
            ds_max[i] = 0.0 if (not np.isfinite(val) or val <= 0.0) else val

        self.samples_s = s_all
        self.q_samp = q
        self.dqds_samp = dqds
        self.ddqdds_samp = ddqdds
        self.ds_max = ds_max

    # ---- Time-parameterization (forward/backward) ----
    def _dds_bounds(self, i: int, ds_i: float) -> Tuple[float, float]:
        """
        Compute conservative bounds on dds at sample i using joint acceleration limits:
          For each joint j:  |ddq_j| = | ddqdds_j * ds^2 + dqds_j * dds | <= max_acc_j
          This gives interval constraints on dds. Intersect across j to get [dds_min, dds_max].
        """
        dq = self.dqds_samp[i]
        ddq = self.ddqdds_samp[i]
        maxa = self.lim.max_acc
        dds_lo, dds_hi = -np.inf, np.inf
        # For each joint: -maxa <= ddq*ds^2 + dq*dds <= maxa
        # => (-maxa - ddq*ds^2)/dq <= dds <= ( maxa - ddq*ds^2)/dq if dq != 0
        # If dq==0: we must have |ddq*ds^2| <= maxa, else infeasible; dds unconstrained by this joint.
        for j in range(self.J):
            A = dq[j]
            B = ddq[j] * ds_i * ds_i
            if abs(A) > NUM_TOL:
                lo = (-maxa[j] - B) / A
                hi = ( maxa[j] - B) / A
                if lo > hi: lo, hi = hi, lo
                dds_lo = max(dds_lo, lo)
                dds_hi = min(dds_hi, hi)
            else:
                # Only check feasibility of curvature term
                if abs(B) > maxa[j] + 1e-12:
                    # No feasible dds satisfies this at given ds_i -> clamp later by reducing ds
                    dds_lo = 1.0
                    dds_hi = -1.0
        return float(dds_lo), float(dds_hi)

    def _forward_backward(self):
        """Classic fwd/bwd pass on s grid with dds interval constraints."""
        s = self.samples_s
        ds = self.ds_max.copy() # start from envelope
        N = s.size

        # forward pass: ensure we can accelerate within dds bounds
        for i in range(N-1):
            ds_i = max(0.0, ds[i])
            dds_lo, dds_hi = self._dds_bounds(i, max(ds_i, 1e-8))
            # Use most positive feasible acceleration (closer to hi)
            dds_use = dds_hi
            ds_next_sq = ds_i*ds_i + 2.0 * dds_use * (s[i+1]-s[i])
            ds_next = math.sqrt(max(0.0, ds_next_sq))
            ds[i+1] = min(ds[i+1], ds_next, self.ds_max[i+1])

        # backward pass: ensure deceleration feasible
        for i in reversed(range(1, N)):
            ds_i = max(0.0, ds[i])
            dds_lo, dds_hi = self._dds_bounds(i, max(ds_i, 1e-8))
            # Use most negative feasible acceleration (closer to lo)
            dds_use = dds_lo
            ds_prev_sq = ds_i*ds_i + 2.0 * dds_use * (s[i-1]-s[i])
            ds_prev = math.sqrt(max(0.0, ds_prev_sq))
            ds[i-1] = min(ds[i-1], ds_prev, self.ds_max[i-1])

        self.ds = ds

    # ---- Public API ----
    def build(self):
        """Generate spline pieces and compute a feasible ds(s)."""
        self._build_pieces()
        self._build_samples()
        self._forward_backward()
    
    def resample(self, dt: float):
        s_grid, ds_grid = self.samples_s, np.maximum(self.ds, 1e-6)
        s_end = s_grid[-1]
        s = [s_grid[0]]
        t = [0.0]
        while s[-1] < s_end - 1e-12:
            ds_cur = float(np.interp(s[-1], s_grid, ds_grid))
            s_next = min(s[-1] + ds_cur * dt, s_end)
            s.append(s_next)
            t.append(t[-1] + dt)
        s = np.asarray(s); t = np.asarray(t)

        # evaluate q, dq/ds, ddq/ds2 along s
        N = s.size
        q = np.zeros((N, self.J))
        dqds = np.zeros_like(q)
        ddqdds = np.zeros_like(q)
        for k in range(N):
            pc = self._piece_at(float(s[k]))
            qi, dqi, ddi = self._eval_piece(pc, float(s[k]))
            q[k], dqds[k], ddqdds[k] = qi, dqi, ddi

        ds_t = np.gradient(s, t, edge_order=2)
        dds_t = np.gradient(ds_t, t, edge_order=2)
        qd = dqds * ds_t[:, None]
        qdd = ddqdds * (ds_t[:, None]**2) + dqds * dds_t[:, None]
        return t, q, qd, qdd

    def minimum_time(self) -> float:
        # integrate dt = ∫ (1/ds) ds over the built profile
        s = self.samples_s
        ds = np.maximum(self.ds, 1e-9)
        return float(np.trapz(1.0 / ds, s))

    def scale_to_duration(self, T_req: float) -> float:
        T_min = self.minimum_time()
        if T_req + 1e-9 < T_min:
            warnings.warn(
                (f"Requested duration {T_req:.3f}s is shorter than the minimum feasible "
                 f"{T_min:.3f}s under current limits. Executing the minimum-time trajectory "
                 f"({T_min:.3f}s)."),
                RuntimeWarning,
                stacklevel=2,
            )
            # Can't go faster without violating limits; leave self.ds as-is (minimum-time).
            return T_min

        # slow down uniformly: T = T_min / k  => choose k = T_min / T_req
        k = T_min / T_req  # slow-down factor
        self.ds *= k
        return T_min
