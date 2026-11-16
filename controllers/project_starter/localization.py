"""Monte Carlo Localization (particle filter) with a LiDAR likelihood field.

Tracking (normal operation)
---------------------------
1. Motion update: every particle is moved by the odometry increment measured
   since the previous update, expressed in the robot frame, plus zero-mean
   Gaussian noise whose standard deviation grows with distance and rotation.
2. Measurement update: a subset of LiDAR beams is projected into the map from
   each particle and scored with a precomputed likelihood field
       p(z | x, m) = z_hit * exp(-d^2 / (2 sigma^2)) + z_rand,
   where ``d`` is the distance from the beam endpoint to the nearest occupied
   map cell.  Beam log-likelihoods are summed and tempered by
   ``effective_beams / beam_count`` because neighbouring beams are correlated.
3. Weights are normalized and the effective sample size
       N_eff = 1 / sum(w_i^2)
   is monitored.  Low-variance resampling runs only when N_eff falls below
   ``resample_ess_fraction * N`` (or when the particle count changes).
4. The pose estimate is the weighted mean of the dominant particle cluster;
   heading uses the circular mean atan2(sum w sin(theta), sum w cos(theta)).

Global localization and kidnapped-robot recovery
-------------------------------------------------
The building has four congruent rooms (including door positions), so a scan
taken inside a room matches up to four poses equally well; only views along
the corridor disambiguate them.  A single particle cloud collapses onto one of
those matches because tiny rasterization differences accumulate, so recovery
uses explicit multiple hypotheses:

* a brute-force scan-matching search scores every free cell (0.1 m) at 36
  headings and keeps the distinct best matches;
* each match becomes a small particle filter of its own (motion + measurement
  + resampling inside the hypothesis);
* hypotheses compete through *decaying* evidence
      e_k = lambda * e_(k-1) + T * (L_best - L_h),
  so a constant small bias saturates at ``bias / (1 - lambda)`` while real
  disambiguating evidence (corridor ends) removes the wrong ones within a few
  updates;
* when one hypothesis remains, is tight, and explains the scan well, its
  particles become the main filter again.

Kidnapping is detected when the converged estimate explains too few beams for
consecutive updates, or when the estimate jumps implausibly far.

Between measurement updates the reported estimate is propagated with the
odometry increment so the controller sees a smooth pose.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from random import Random

from occupancy_grid import distance_field
from odometry import Pose2D, wrap_angle


DEFAULT_SETTINGS: dict = {
    "mode": "mcl",
    "initial_belief": "gaussian",
    "initial_position_std_m": 0.45,
    "initial_heading_std_rad": 0.50,
    "initial_region": None,
    "particle_count_recommendation": 300,
    "beams_per_update": 40,
    "effective_beams": 10.0,
    "sigma_hit_m": 0.05,
    "z_hit": 1.0,
    "z_rand": 0.05,
    "field_resolution_m": 0.05,
    "field_max_distance_m": 0.6,
    "max_beam_range_m": 3.8,
    "update_min_translation_m": 0.02,
    "update_min_rotation_rad": 0.05,
    "update_max_steps": 12,
    "motion_noise_translation": 0.08,
    "motion_noise_translation_per_rad": 0.01,
    "motion_noise_rotation": 0.08,
    "motion_noise_rotation_per_m": 0.25,
    "motion_noise_floor_m": 0.002,
    "motion_noise_floor_rad": 0.004,
    "resample_ess_fraction": 0.5,
    "estimate_cluster_radius_m": 0.35,
    "converged_position_std_m": 0.10,
    "converged_heading_std_rad": 0.12,
    "inlier_distance_m": 0.08,
    "accept_inlier_ratio": 0.90,
    "accept_window_updates": 10,
    "lost_inlier_ratio": 0.45,
    "lost_consecutive_updates": 4,
    "kidnap_inlier_ratio": 0.25,
    "kidnap_consecutive_updates": 2,
    "kidnap_jump_m": 0.5,
    "augmented_alpha_slow": 0.02,
    "augmented_alpha_fast": 0.25,
    "augmented_injection_max": 0.0,
    "free_space_clearance_m": 0.05,
    "unconverged_search_after_updates": 250,
    "search_resolution_m": 0.10,
    "search_headings": 36,
    "search_beams": 24,
    "search_max_hypotheses": 12,
    "search_score_margin": 12.0,
    "search_separation_m": 0.30,
    "hypothesis_particles": 100,
    "hypothesis_evidence_decay": 0.85,
    "hypothesis_prune_evidence": 6.0,
    "hypothesis_min_inlier_ratio": 0.5,
    "hypothesis_bad_updates": 3,
    "research_after_bad_updates": 25,
    "ambiguous_commit_travel_m": 4.0,
    "lidar_offset_x_m": -0.0095,
    "seed": 185,
}


@dataclass
class Particle:
    x: float
    y: float
    theta: float
    weight: float


@dataclass
class Hypothesis:
    """One candidate pose cluster during global localization."""

    particles: list[Particle]
    estimate: tuple[float, float, float]
    evidence: float = 0.0
    best_log_likelihood: float = 0.0
    inlier_ratio: float = 1.0
    bad_updates: int = 0
    label: int = 0
    history: deque = field(default_factory=lambda: deque(maxlen=10))


def relative_motion(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Return ``end`` expressed in the frame of ``start`` as (dx, dy, dtheta)."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    c = math.cos(start[2])
    s = math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(end[2] - start[2])


def compose(
    pose: tuple[float, float, float], motion: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Apply a robot-frame motion (dx, dy, dtheta) to a pose."""
    c = math.cos(pose[2])
    s = math.sin(pose[2])
    return (
        pose[0] + c * motion[0] - s * motion[1],
        pose[1] + s * motion[0] + c * motion[1],
        wrap_angle(pose[2] + motion[2]),
    )


class StudentLocalizer:
    """Monte Carlo Localization; ``mode="odometry_baseline"`` passes odometry through."""

    def __init__(
        self,
        initial_pose: tuple[float, float, float],
        particle_count: int = 300,
        settings: dict | None = None,
        seed: int | None = None,
    ) -> None:
        self.settings = dict(DEFAULT_SETTINGS)
        if settings:
            self.settings.update({key: value for key, value in settings.items() if key in DEFAULT_SETTINGS})
        self.settings["particle_count_recommendation"] = int(particle_count)
        s = self.settings
        self.mode = str(s["mode"])
        self.initial_pose = tuple(float(value) for value in initial_pose)
        self.particle_count = int(particle_count)
        self.target_count = self.particle_count
        self.random = Random(int(s["seed"] if seed is None else seed))
        self.particles: list[Particle] = []
        self.hypotheses: list[Hypothesis] = []
        self.followed_label = -1
        self.next_label = 0
        self.pending_search: tuple | None = None

        self.anchor_odometry: tuple[float, float, float] | None = None
        self.anchor_estimate: tuple[float, float, float] = self.initial_pose
        self.last_scan: list[tuple[float, float]] = []
        self.steps_since_update = 0
        self.updated_this_step = False
        self.update_count = 0
        self.converged = self.mode != "mcl"
        self.recovering = False
        self.recovery_events = 0
        self.search_count = 0
        self.position_spread = 0.0
        self.heading_spread = 0.0
        self.effective_sample_size = float(self.particle_count)
        self.last_inlier_ratio = 1.0
        self.last_jump = 0.0
        self.bad_updates = 0
        self.severe_updates = 0
        self.unconverged_updates = 0
        self.research_bad_updates = 0
        self.search_travel = 0.0
        self.ambiguous_commits = 0
        self.prior_pose: tuple[float, float, float] | None = None
        self.inlier_window: deque[float] = deque(maxlen=int(s["accept_window_updates"]))
        self.w_slow = 0.0
        self.w_fast = 0.0

    # ------------------------------------------------------------------
    # Map model and initial belief
    # ------------------------------------------------------------------
    def initialize(self, occupancy_grid) -> None:
        """Build the likelihood field and draw the initial belief."""
        if self.mode != "mcl":
            self.particles = []
            return
        s = self.settings
        resolution = float(s["field_resolution_m"])
        max_distance = float(s["field_max_distance_m"])
        # Map cells overlapping a wall are fully occupied, which fattens walls
        # by up to half a cell; a quarter-cell square balances that bias.
        half_size = 0.25 * occupancy_grid.resolution
        self.distance, self.field_nx, self.field_ny = distance_field(
            occupancy_grid, resolution, half_size, max_distance
        )
        self.field_resolution = resolution
        self.field_origin_x = occupancy_grid.origin_x
        self.field_origin_y = occupancy_grid.origin_y
        sigma = float(s["sigma_hit_m"])
        z_hit = float(s["z_hit"])
        z_rand = float(s["z_rand"])
        self.log_field = [
            math.log(z_hit * math.exp(-0.5 * (d / sigma) ** 2) + z_rand) for d in self.distance
        ]
        self.log_outside = math.log(z_rand)
        clearance = float(s["free_space_clearance_m"])
        self.free_cells = [index for index, value in enumerate(self.distance) if value >= clearance]
        self.reset_belief(str(s["initial_belief"]))

    def _field_index(self, x: float, y: float) -> int | None:
        fx = (x - self.field_origin_x) / self.field_resolution
        fy = (y - self.field_origin_y) / self.field_resolution
        if fx < 0.0 or fy < 0.0 or fx >= self.field_nx or fy >= self.field_ny:
            return None
        return int(fy) * self.field_nx + int(fx)

    def _is_free(self, x: float, y: float) -> bool:
        index = self._field_index(x, y)
        return index is not None and self.distance[index] >= float(self.settings["free_space_clearance_m"])

    def _random_free_particle(self) -> Particle:
        index = self.random.choice(self.free_cells)
        j, i = divmod(index, self.field_nx)
        x = self.field_origin_x + (i + self.random.random()) * self.field_resolution
        y = self.field_origin_y + (j + self.random.random()) * self.field_resolution
        return Particle(x, y, self.random.uniform(-math.pi, math.pi), 1.0)

    def reset_belief(self, belief: str) -> None:
        """Initial belief: ``gaussian`` around the configured start pose,
        ``region`` (uniform over ``initial_region``, any heading) or ``global``
        (whole free space).  Region and global beliefs are realized with the
        scan-matching hypothesis search on the first LiDAR scan."""
        s = self.settings
        self.converged = False
        self.unconverged_updates = 0
        self.inlier_window.clear()
        if belief == "gaussian":
            x0, y0, theta0 = self.initial_pose
            position_std = float(s["initial_position_std_m"])
            heading_std = float(s["initial_heading_std_rad"])
            particles: list[Particle] = []
            attempts = 0
            while len(particles) < self.particle_count and attempts < 100 * self.particle_count:
                attempts += 1
                x = self.random.gauss(x0, position_std)
                y = self.random.gauss(y0, position_std)
                if self._is_free(x, y):
                    particles.append(Particle(x, y, wrap_angle(self.random.gauss(theta0, heading_std)), 1.0))
            if not particles:
                raise RuntimeError("Initial belief contains no free-space particle")
            self._normalize_uniform(particles)
            self.particles = particles
            self.target_count = len(particles)
            self.anchor_estimate = self._cluster_estimate(particles, None)
        elif belief == "region":
            region = s["initial_region"]
            if not region or len(region) != 4:
                raise ValueError("initial_belief 'region' requires initial_region [xmin, xmax, ymin, ymax]")
            self.pending_search = tuple(float(value) for value in region)
            self.particles = []
        elif belief == "global":
            self.pending_search = ()
            self.particles = []
        else:
            raise ValueError(f"Unknown initial belief {belief!r}")

    # ------------------------------------------------------------------
    # Generic particle operations
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_uniform(particles: list[Particle]) -> None:
        weight = 1.0 / len(particles)
        for particle in particles:
            particle.weight = weight

    def _move(self, particles: list[Particle], motion: tuple[float, float, float]) -> None:
        s = self.settings
        dx, dy, dtheta = motion
        translation = math.hypot(dx, dy)
        sigma_t = (
            float(s["motion_noise_translation"]) * translation
            + float(s["motion_noise_translation_per_rad"]) * abs(dtheta)
            + float(s["motion_noise_floor_m"])
        )
        sigma_r = (
            float(s["motion_noise_rotation"]) * abs(dtheta)
            + float(s["motion_noise_rotation_per_m"]) * translation
            + float(s["motion_noise_floor_rad"])
        )
        gauss = self.random.gauss
        for particle in particles:
            mx = dx + gauss(0.0, sigma_t)
            my = dy + gauss(0.0, 0.5 * sigma_t)
            c = math.cos(particle.theta)
            sn = math.sin(particle.theta)
            particle.x += c * mx - sn * my
            particle.y += sn * mx + c * my
            particle.theta = wrap_angle(particle.theta + dtheta + gauss(0.0, sigma_r))

    def _select_beams(self, scan_points_robot, count: int | None = None) -> list[tuple[float, float]]:
        s = self.settings
        stride = max(1, len(scan_points_robot) // int(count or s["beams_per_update"]))
        offset = float(s["lidar_offset_x_m"])
        max_range_sq = float(s["max_beam_range_m"]) ** 2
        return [(x + offset, y) for x, y in scan_points_robot[::stride] if x * x + y * y <= max_range_sq]

    def _log_likelihood(self, x: float, y: float, theta: float, beams) -> float:
        log_field = self.log_field
        outside = self.log_outside
        nx = self.field_nx
        ny = self.field_ny
        ox = self.field_origin_x
        oy = self.field_origin_y
        inverse = 1.0 / self.field_resolution
        c = math.cos(theta)
        s = math.sin(theta)
        total = 0.0
        for bx, by in beams:
            fx = (x + c * bx - s * by - ox) * inverse
            fy = (y + s * bx + c * by - oy) * inverse
            if 0.0 <= fx < nx and 0.0 <= fy < ny:
                total += log_field[int(fy) * nx + int(fx)]
            else:
                total += outside
        return total

    def _reweight(self, particles: list[Particle], scores: list[float], temperature: float) -> float:
        """Bayes update in log space; returns the effective sample size."""
        log_weights = [
            math.log(max(p.weight, 1e-300)) + temperature * score for p, score in zip(particles, scores)
        ]
        peak = max(log_weights)
        weights = [math.exp(value - peak) for value in log_weights]
        total = sum(weights)
        for particle, weight in zip(particles, weights):
            particle.weight = weight / total
        return 1.0 / sum(p.weight * p.weight for p in particles)

    def _resample(self, particles: list[Particle], count: int, inject: int = 0) -> list[Particle]:
        """Low-variance (systematic) resampling plus optional random particles."""
        draw = max(1, count - inject)
        step = 1.0 / draw
        position = self.random.random() * step
        cumulative = particles[0].weight
        index = 0
        new: list[Particle] = []
        for _ in range(draw):
            while position > cumulative and index < len(particles) - 1:
                index += 1
                cumulative += particles[index].weight
            p = particles[index]
            new.append(Particle(p.x, p.y, p.theta, 1.0))
            position += step
        new.extend(self._random_free_particle() for _ in range(inject))
        self._normalize_uniform(new)
        return new

    @staticmethod
    def _spread(particles: list[Particle]) -> tuple[float, float]:
        total = sum(p.weight for p in particles)
        mean_x = sum(p.weight * p.x for p in particles) / total
        mean_y = sum(p.weight * p.y for p in particles) / total
        variance = sum(p.weight * ((p.x - mean_x) ** 2 + (p.y - mean_y) ** 2) for p in particles) / total
        sin_sum = sum(p.weight * math.sin(p.theta) for p in particles) / total
        cos_sum = sum(p.weight * math.cos(p.theta) for p in particles) / total
        resultant = min(1.0, math.hypot(sin_sum, cos_sum))
        return math.sqrt(max(0.0, variance)), math.sqrt(-2.0 * math.log(max(resultant, 1e-12)))

    def _cluster(self, particles, center) -> tuple[list[Particle], float]:
        radius_sq = float(self.settings["estimate_cluster_radius_m"]) ** 2
        cluster = [
            p
            for p in particles
            if (p.x - center[0]) ** 2 + (p.y - center[1]) ** 2 <= radius_sq
            and abs(wrap_angle(p.theta - center[2])) <= 0.6
        ]
        return cluster, sum(p.weight for p in cluster)

    def _cluster_estimate(self, particles, previous) -> tuple[float, float, float]:
        """Weighted mean of the dominant cluster (circular mean for heading).

        The estimate stays on the cluster around ``previous`` unless another
        cluster is 1.5 times heavier, so it does not chase noise between modes.
        """
        best = max(particles, key=lambda p: p.weight)
        cluster, total = self._cluster(particles, (best.x, best.y, best.theta))
        if previous is not None:
            kept, kept_total = self._cluster(particles, previous)
            if kept and 1.5 * kept_total >= total:
                cluster, total = kept, kept_total
        x = sum(p.weight * p.x for p in cluster) / total
        y = sum(p.weight * p.y for p in cluster) / total
        theta = math.atan2(
            sum(p.weight * math.sin(p.theta) for p in cluster),
            sum(p.weight * math.cos(p.theta) for p in cluster),
        )
        return x, y, theta

    def inlier_ratio(self, pose: tuple[float, float, float], scan_points_robot) -> float:
        """Fraction of beams ending within ``inlier_distance_m`` of a mapped wall."""
        beams = self._select_beams(scan_points_robot)
        if not beams:
            return 0.0
        threshold = float(self.settings["inlier_distance_m"])
        c = math.cos(pose[2])
        s = math.sin(pose[2])
        inliers = 0
        for bx, by in beams:
            index = self._field_index(pose[0] + c * bx - s * by, pose[1] + s * bx + c * by)
            if index is not None and self.distance[index] <= threshold:
                inliers += 1
        return inliers / len(beams)

    # ------------------------------------------------------------------
    # Global search (scan matching) and multi-hypothesis recovery
    # ------------------------------------------------------------------
    def scan_search(self, scan_points_robot, region=None) -> list[tuple[float, float, float, float]]:
        """Brute-force scan matching over free space.

        Every free cell on a ``search_resolution_m`` lattice is scored at
        ``search_headings`` headings with the likelihood field, using integer
        index offsets precomputed per heading.  Returns distinct maxima as
        ``(score, x, y, theta)``, best first.
        """
        s = self.settings
        beams = self._select_beams(scan_points_robot, int(s["search_beams"]))
        if len(beams) < 8:
            return []
        resolution = self.field_resolution
        nx = self.field_nx
        ny = self.field_ny
        log_field = self.log_field
        outside = self.log_outside
        headings = int(s["search_headings"])
        offsets_by_heading = []
        for k in range(headings):
            theta = -math.pi + k * 2.0 * math.pi / headings
            c = math.cos(theta)
            sn = math.sin(theta)
            offsets_by_heading.append(
                (
                    theta,
                    [
                        (int(round((c * bx - sn * by) / resolution)), int(round((sn * bx + c * by) / resolution)))
                        for bx, by in beams
                    ],
                )
            )
        step = max(1, int(round(float(s["search_resolution_m"]) / resolution)))
        clearance = float(s["free_space_clearance_m"])
        candidates: list[tuple[float, float, float, float]] = []
        for j in range(0, ny, step):
            y = self.field_origin_y + (j + 0.5) * resolution
            for i in range(0, nx, step):
                if self.distance[j * nx + i] < clearance:
                    continue
                x = self.field_origin_x + (i + 0.5) * resolution
                if region and not (region[0] <= x <= region[1] and region[2] <= y <= region[3]):
                    continue
                best_score = -math.inf
                best_theta = 0.0
                for theta, offsets in offsets_by_heading:
                    total = 0.0
                    for di, dj in offsets:
                        ii = i + di
                        jj = j + dj
                        if 0 <= ii < nx and 0 <= jj < ny:
                            total += log_field[jj * nx + ii]
                        else:
                            total += outside
                    if total > best_score:
                        best_score = total
                        best_theta = theta
                candidates.append((best_score, x, y, best_theta))
        candidates.sort(reverse=True)
        if not candidates:
            return []
        margin = float(s["search_score_margin"])
        separation_sq = float(s["search_separation_m"]) ** 2
        top = candidates[0][0]
        selected: list[tuple[float, float, float, float]] = []
        for candidate in candidates:
            if candidate[0] < top - margin or len(selected) >= int(s["search_max_hypotheses"]):
                break
            if all((candidate[1] - c[1]) ** 2 + (candidate[2] - c[2]) ** 2 > separation_sq for c in selected):
                selected.append(candidate)
        return selected

    def _begin_search(self, scan_points_robot, region=None) -> bool:
        """Replace the belief by one small particle filter per scan match."""
        matches = self.scan_search(scan_points_robot, region)
        if not matches:
            return False
        per = int(self.settings["hypothesis_particles"])
        hypotheses = []
        for _, x, y, theta in matches:
            particles = []
            while len(particles) < per:
                px = self.random.gauss(x, 0.05)
                py = self.random.gauss(y, 0.05)
                if self._is_free(px, py) or len(particles) == 0:
                    particles.append(Particle(px, py, wrap_angle(self.random.gauss(theta, 0.08)), 1.0))
            self._normalize_uniform(particles)
            hypotheses.append(Hypothesis(particles, (x, y, theta), label=self.next_label))
            self.next_label += 1
        self.hypotheses = hypotheses
        self.followed_label = self._preferred(hypotheses).label
        self.search_travel = 0.0
        self.recovering = True
        self.converged = False
        self.unconverged_updates = 0
        self.research_bad_updates = 0
        self.search_count += 1
        self.inlier_window.clear()
        self._sync_particles_from_hypotheses()
        return True

    def start_global_recovery(self) -> None:
        """Kidnapped-robot recovery: global multi-hypothesis search."""
        self.prior_pose = self.anchor_estimate
        self.recovery_events += 1
        self.bad_updates = 0
        self.severe_updates = 0
        self.w_slow = self.w_fast = 0.0
        if not self._begin_search(self.last_scan):
            self.pending_search = ()

    def _sync_particles_from_hypotheses(self) -> None:
        share = 1.0 / max(1, len(self.hypotheses))
        self.particles = [
            Particle(p.x, p.y, p.theta, p.weight * share) for h in self.hypotheses for p in h.particles
        ]
        followed = self._followed()
        if followed is not None:
            self.anchor_estimate = followed.estimate

    def _preferred(self, hypotheses: list[Hypothesis]) -> Hypothesis:
        """Lowest evidence against; ties go to the smallest displacement from
        the last confident estimate (a disturbance is more likely local)."""
        lowest = min(h.evidence for h in hypotheses)
        tied = [h for h in hypotheses if h.evidence <= lowest + 0.5]
        if self.prior_pose is None:
            return tied[0]
        px, py, _ = self.prior_pose
        return min(tied, key=lambda h: (h.estimate[0] - px) ** 2 + (h.estimate[1] - py) ** 2)

    def _followed(self) -> Hypothesis | None:
        for hypothesis in self.hypotheses:
            if hypothesis.label == self.followed_label:
                return hypothesis
        return None

    def _update_hypotheses(self, motion, scan_points_robot) -> None:
        s = self.settings
        beams = self._select_beams(scan_points_robot)
        if len(beams) < 5:
            return
        temperature = float(s["effective_beams"]) / len(beams)
        count = int(s["hypothesis_particles"])
        self.search_travel += math.hypot(motion[0], motion[1])
        for h in self.hypotheses:
            self._move(h.particles, motion)
            scores = [self._log_likelihood(p.x, p.y, p.theta, beams) for p in h.particles]
            h.best_log_likelihood = max(scores)
            ess = self._reweight(h.particles, scores, temperature)
            if ess < float(s["resample_ess_fraction"]) * len(h.particles):
                h.particles = self._resample(h.particles, count)
            h.estimate = self._cluster_estimate(h.particles, compose(h.estimate, motion))
            h.inlier_ratio = self.inlier_ratio(h.estimate, scan_points_robot)
            h.history.append(h.inlier_ratio)

        best = max(h.best_log_likelihood for h in self.hypotheses)
        decay = float(s["hypothesis_evidence_decay"])
        for h in self.hypotheses:
            h.evidence = decay * h.evidence + temperature * (best - h.best_log_likelihood)
            if h.inlier_ratio < float(s["hypothesis_min_inlier_ratio"]):
                h.bad_updates += 1
            else:
                h.bad_updates = 0
        prune = float(s["hypothesis_prune_evidence"])
        bad_limit = int(s["hypothesis_bad_updates"])
        survivors = [h for h in self.hypotheses if h.evidence <= prune and h.bad_updates < bad_limit]
        if not survivors:
            survivors = [min(self.hypotheses, key=lambda h: h.evidence)]
        if len(survivors) < len(self.hypotheses):
            # The commit rule needs a long drive without any new evidence.
            self.search_travel = 0.0
        self.hypotheses = survivors
        if self._followed() is None:
            self.followed_label = self._preferred(survivors).label

        # Every hypothesis disagrees with the scan: the true pose was missed.
        # (Correct tracking dips to about 0.6 next to the unmapped blocker, so
        # use the same threshold as kidnap detection.)
        if max(h.inlier_ratio for h in survivors) < float(s["lost_inlier_ratio"]):
            self.research_bad_updates += 1
            if self.research_bad_updates >= int(s["research_after_bad_updates"]):
                self._begin_search(scan_points_robot)
                return
        else:
            self.research_bad_updates = 0

        self._sync_particles_from_hypotheses()
        self.position_spread, self.heading_spread = self._spread(self._followed().particles)
        self.effective_sample_size = float(len(self.particles))

    # ------------------------------------------------------------------
    # Filter update
    # ------------------------------------------------------------------
    def update(
        self,
        odometry_pose: Pose2D,
        delta_motion: tuple[float, float],
        scan_points_robot: list[tuple[float, float]],
        occupancy_grid,
    ) -> Pose2D:
        """Return the current pose estimate (MCL or odometry pass-through)."""
        _ = delta_motion, occupancy_grid
        self.updated_this_step = False
        if self.mode != "mcl":
            return Pose2D(odometry_pose.x, odometry_pose.y, odometry_pose.theta)

        odometry = (odometry_pose.x, odometry_pose.y, odometry_pose.theta)
        if self.anchor_odometry is None:
            self.anchor_odometry = odometry
        if scan_points_robot:
            self.last_scan = scan_points_robot
        if self.pending_search is not None and len(scan_points_robot) >= 10:
            region = self.pending_search or None
            self.pending_search = None
            if self._begin_search(scan_points_robot, region):
                self.anchor_odometry = odometry
            else:
                self.pending_search = region or ()
        if not self.particles:
            return Pose2D(*compose(self.anchor_estimate, relative_motion(self.anchor_odometry, odometry)))

        motion = relative_motion(self.anchor_odometry, odometry)
        self.steps_since_update += 1
        s = self.settings
        due = (
            math.hypot(motion[0], motion[1]) >= float(s["update_min_translation_m"])
            or abs(motion[2]) >= float(s["update_min_rotation_rad"])
            or self.steps_since_update >= int(s["update_max_steps"])
        )
        if due and len(scan_points_robot) >= 10:
            predicted = compose(self.anchor_estimate, motion)
            self.anchor_odometry = odometry
            if self.hypotheses:
                self._update_hypotheses(motion, scan_points_robot)
                if self.hypotheses:
                    self.anchor_estimate = self._followed().estimate
            else:
                self._move(self.particles, motion)
                self._measurement_update(scan_points_robot)
                self.anchor_estimate = self._cluster_estimate(self.particles, predicted)
            self.last_jump = math.hypot(
                self.anchor_estimate[0] - predicted[0], self.anchor_estimate[1] - predicted[1]
            )
            self._check_health(scan_points_robot)
            self.steps_since_update = 0
            self.updated_this_step = True
            self.update_count += 1
            motion = (0.0, 0.0, 0.0)
        return Pose2D(*compose(self.anchor_estimate, motion))

    def _measurement_update(self, scan_points_robot) -> None:
        beams = self._select_beams(scan_points_robot)
        if len(beams) < 5:
            return
        s = self.settings
        temperature = float(s["effective_beams"]) / len(beams)
        scores = [self._log_likelihood(p.x, p.y, p.theta, beams) for p in self.particles]
        # Augmented-MCL bookkeeping uses the per-beam geometric-mean likelihood.
        average = sum(p.weight * math.exp(score / len(beams)) for p, score in zip(self.particles, scores))
        if self.w_slow <= 0.0:
            self.w_slow = self.w_fast = average
        else:
            self.w_slow += float(s["augmented_alpha_slow"]) * (average - self.w_slow)
            self.w_fast += float(s["augmented_alpha_fast"]) * (average - self.w_fast)
        self.effective_sample_size = self._reweight(self.particles, scores, temperature)
        self.position_spread, self.heading_spread = self._spread(self.particles)

        if self.converged and self.target_count > self.particle_count:
            self.target_count = self.particle_count
        if (
            self.effective_sample_size < float(s["resample_ess_fraction"]) * len(self.particles)
            or self.target_count != len(self.particles)
        ):
            inject = 0
            if self.converged and self.w_slow > 0.0:
                fraction = min(float(s["augmented_injection_max"]), max(0.0, 1.0 - self.w_fast / self.w_slow))
                inject = int(fraction * self.target_count)
            self.particles = self._resample(self.particles, self.target_count, inject)
            self.effective_sample_size = float(len(self.particles))

    # ------------------------------------------------------------------
    # Convergence, acceptance, and kidnap detection
    # ------------------------------------------------------------------
    def _check_health(self, scan_points_robot) -> None:
        s = self.settings
        self.last_inlier_ratio = self.inlier_ratio(self.anchor_estimate, scan_points_robot)
        self.inlier_window.append(self.last_inlier_ratio)
        tight = (
            self.position_spread <= float(s["converged_position_std_m"])
            and self.heading_spread <= float(s["converged_heading_std_rad"])
        )
        if not self.converged:
            self.unconverged_updates += 1
            single = len(self.hypotheses) <= 1
            window_full = len(self.inlier_window) == self.inlier_window.maxlen
            mean_inliers = sum(self.inlier_window) / len(self.inlier_window)
            if single and tight and window_full and mean_inliers >= float(s["accept_inlier_ratio"]):
                self._accept()
            elif (
                len(self.hypotheses) > 1
                and self.search_travel >= float(s["ambiguous_commit_travel_m"])
                and tight
                and window_full
                and mean_inliers >= float(s["accept_inlier_ratio"])
                and all(min(h.history) >= float(s["accept_inlier_ratio"]) - 0.1 for h in self.hypotheses)
            ):
                # Several hypotheses still explain every scan after driving
                # through the corridor: the world is symmetric for them (for
                # example 180 deg about the origin with the blocker at
                # position 1).  No sensor can separate them, so commit to the
                # followed hypothesis instead of stalling the mission.
                followed = self._followed()
                self.hypotheses = [followed]
                self.ambiguous_commits += 1
                self._accept()
            elif (
                not self.hypotheses
                and self.unconverged_updates >= int(s["unconverged_search_after_updates"])
            ):
                # The Gaussian initial belief did not contain the true pose.
                self._begin_search(scan_points_robot)
            return
        if self.last_jump >= float(s["kidnap_jump_m"]):
            self.start_global_recovery()
            return
        self.severe_updates = self.severe_updates + 1 if self.last_inlier_ratio < float(s["kidnap_inlier_ratio"]) else 0
        self.bad_updates = self.bad_updates + 1 if self.last_inlier_ratio < float(s["lost_inlier_ratio"]) else 0
        if (
            self.severe_updates >= int(s["kidnap_consecutive_updates"])
            or self.bad_updates >= int(s["lost_consecutive_updates"])
        ):
            self.start_global_recovery()

    def _accept(self) -> None:
        if self.hypotheses:
            particles = self.hypotheses[0].particles
            self.particles = self._resample(particles, self.particle_count)
            self.hypotheses = []
        self.target_count = self.particle_count
        self.converged = True
        self.recovering = False
        self.unconverged_updates = 0
        self.bad_updates = 0
        self.severe_updates = 0

    def sample_particles(self, count: int = 120) -> list[tuple[float, float, float]]:
        stride = max(1, len(self.particles) // max(1, count))
        return [(p.x, p.y, p.theta) for p in self.particles[::stride]]
