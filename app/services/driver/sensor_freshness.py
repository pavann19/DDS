"""Producer-clock freshness and latched recovery for simulated sensor feeds."""
from dataclasses import dataclass
import math

from app.services.interfaces import SensorHealth


@dataclass(frozen=True)
class SensorFreshnessStatus:
    monitored: bool
    ready: bool
    reason: str
    forward_age_s: float | None
    surround_age_s: float | None
    recovery_samples: int
    camera_required: bool = False
    camera_age_s: float | None = None
    camera_recovery_samples: int = 0
    camera_view_health: tuple = ()


def _valid_time(value):
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError):
        return False


class SensorFreshnessGuard:
    MAX_AGE_S = .15
    RECOVERY_SAMPLES = 3

    def __init__(self):
        self.reset()

    def reset(self):
        self._view_guards = {}
        self._view_health = ()
        self._camera_required = False
        self._camera_high_water = None
        self._camera_last_sample = None
        self._camera_recovery_samples = 0
        self._last_time = None
        self._last_sample = None
        self._last_captures = None
        self._contract_seen = False
        self._recovery_samples = 0
        self.status = SensorFreshnessStatus(False, False, 'sensor_unmonitored', None, None, 0)

    def evaluate(self, now: float, health: SensorHealth | None):
        if health is None:
            self._camera_recovery_samples = 0
            for guard in self._view_guards.values(): guard._camera_recovery_samples = 0
            self._recovery_samples = 0
            self._last_sample = None
            if self._camera_required and self._view_guards:
                self._view_health = tuple((name, None, guard._camera_recovery_samples, False)
                                          for name, guard in self._view_guards.items())
            self.status = SensorFreshnessStatus(self._contract_seen, False,
                'sensor_missing' if self._contract_seen else 'sensor_unmonitored', None, None, 0,
                self._camera_required, None, 0, self._view_health)
            return self.status
        self._contract_seen = True
        self._camera_required = health.camera_required
        if health.camera_required and health.camera_required_views and not self._view_health:
            self._view_health = tuple((name, None, 0, False) for name in health.camera_required_views)
        if not _valid_time(now):
            self._camera_recovery_samples = 0
            for guard in self._view_guards.values(): guard._camera_recovery_samples = 0
            self._view_health = tuple((name, None, 0, False) for name, *_ in self._view_health)
            self._recovery_samples = 0
            self._last_sample = None
            self.status = SensorFreshnessStatus(True, False, 'sensor_timestamp_invalid', None, None, 0,
                self._camera_required, None, 0, self._view_health)
            return self.status
        if self._last_time is not None and now < self._last_time:
            self.reset()
        self._contract_seen = True
        self._camera_required = health.camera_required
        self._last_time = now
        ages = []
        reason = None
        for stamp in (health.forward_capture_time_s, health.surround_capture_time_s):
            if stamp is None:
                ages.append(None)
                reason = reason or 'sensor_missing'
            elif not _valid_time(stamp) or stamp > now + 1e-6:
                ages.append(None)
                reason = 'sensor_timestamp_invalid'
            else:
                age = max(0., now - stamp)
                ages.append(age)
                if age > self.MAX_AGE_S + 1e-9:
                    reason = reason or 'sensor_stale'
        captures = (health.forward_capture_time_s, health.surround_capture_time_s)
        if reason is None and self._last_captures is not None:
            if any(new < old - 1e-9 for new, old in zip(captures, self._last_captures)
                   if old is not None):
                reason = 'sensor_timestamp_invalid'
        if all(stamp is not None and _valid_time(stamp) for stamp in captures):
            self._last_captures = captures
        if reason is not None:
            self._recovery_samples = 0
            self._last_sample = None
        else:
            # Both feeds must advance. min(timestamp) can advance twice
            # when the faster feed catches up to one surround acquisition.
            sample = captures
            if self._last_sample is None or all(new > old + 1e-9
                                                for new, old in zip(sample, self._last_sample)):
                self._recovery_samples = min(self.RECOVERY_SAMPLES, self._recovery_samples + 1)
                self._last_sample = sample
            if self._recovery_samples < self.RECOVERY_SAMPLES:
                reason = 'sensor_recovering'
        camera_age, camera_reason = self._evaluate_camera(now, health)
        reason = reason or camera_reason
        self.status = SensorFreshnessStatus(True, reason is None, reason or 'sensor_ready',
                                            ages[0], ages[1], self._recovery_samples,
                                            health.camera_required, camera_age,
                                            self._camera_recovery_samples, self._view_health)
        return self.status

    def _evaluate_camera(self, now, health):
        if health.camera_required and health.camera_required_views:
            from dataclasses import replace
            stamps=dict(health.camera_capture_times_s)
            statuses=[]
            reasons=[]
            for name in health.camera_required_views:
                guard=self._view_guards.setdefault(name,SensorFreshnessGuard())
                age,reason=guard._evaluate_camera(now,replace(health,camera_required_views=(),
                    camera_capture_time_s=stamps.get(name)))
                statuses.append((name,age,guard._camera_recovery_samples,reason is None))
                if reason:reasons.append(reason)
            self._view_health=tuple(statuses)
            self._camera_recovery_samples=min(row[2] for row in statuses)
            ages=[row[1] for row in statuses if row[1] is not None]
            return max(ages) if ages else None, next((r for r in reasons if r!='sensor_camera_recovering'),reasons[0] if reasons else None)
        self._view_health=()
        self._view_guards={}
        if not health.camera_required:
            self._camera_high_water = None
            self._camera_last_sample = None
            self._camera_recovery_samples = 0
            return None, None
        stamp = health.camera_capture_time_s
        age = None
        if stamp is None:
            reason = 'sensor_camera_missing'
        elif not _valid_time(stamp) or stamp < 0 or stamp > now + 1e-6:
            reason = 'sensor_camera_timestamp_invalid'
        elif self._camera_high_water is not None and stamp < self._camera_high_water - 1e-9:
            reason = 'sensor_camera_timestamp_invalid'
        else:
            age = max(0., now-stamp)
            self._camera_high_water = stamp
            reason = 'sensor_camera_stale' if age > .35 + 1e-9 else None
        if reason:
            self._camera_recovery_samples = 0
            # Retain the last counted capture: replay cannot restore health.
            return age, reason
        if self._camera_last_sample is not None and stamp-self._camera_last_sample > .35 + 1e-9:
            self._camera_recovery_samples = 0
        if self._camera_last_sample is None or stamp > self._camera_last_sample + 1e-9:
            self._camera_recovery_samples = min(self.RECOVERY_SAMPLES,
                                                self._camera_recovery_samples + 1)
            self._camera_last_sample = stamp
        return age, ('sensor_camera_recovering' if self._camera_recovery_samples < self.RECOVERY_SAMPLES else None)
