from dataclasses import dataclass
from typing import Callable, Generic, Optional, TypeVar


T = TypeVar("T")
ClipFn = Callable[[T, T, T], T]


def _zeros_like(x: T) -> T:
    return x - x


def _neg(x: T) -> T:
    return x * (-1)


def _scalar_clip(x: T, lo: T, hi: T) -> T:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _default_clip(x: T, lo: T, hi: T) -> T:
    clip_m = getattr(x, "clip", None)
    if callable(clip_m):
        try:
            return clip_m(lo, hi)
        except TypeError:
            try:
                return clip_m(min=lo, max=hi)
            except TypeError:
                pass
    return _scalar_clip(x, lo, hi)


@dataclass
class AccelLimitedSmoother(Generic[T]):
    q: T
    v_max: T
    a_max: T
    dt: float
    tau_sec: float = 0.2
    v: Optional[T] = None
    clip_fn: Optional[ClipFn] = None

    def __post_init__(self) -> None:
        if self.v is None:
            self.v = _zeros_like(self.q)
        if self.clip_fn is None:
            self.clip_fn = _default_clip

    def step(self, q_target: T) -> tuple[T, T]:
        e = q_target - self.q
        v_goal = self.clip_fn(
            e / self.tau_sec,
            _neg(self.v_max),
            self.v_max,
        )
        a_dt = self.a_max * self.dt
        dv = v_goal - self.v
        dv = self.clip_fn(dv, _neg(a_dt), a_dt)
        self.v = self.v + dv
        self.q = self.q + self.v * self.dt
        return self.q, self.v
