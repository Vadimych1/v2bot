from miniros import get_logger
from enum import Enum
import time, math

l = get_logger("motion_controller.state_machine")

class State(Enum):
    IDLE = 0
    FOLLOW_PATH = 1
    ADJUST = 2
    RECOVERY = 3
    FAILURE = 4
    MANUAL = 5

class RecoveryBehavior(Enum):
    ROTATE = 0
    BACK = 1
    BACK_AND_ROTATE = 2

class StateMachine:
    def __init__(
        self, 
        max_no_progress_time: float = 10.0,
        progress_threshold: float = 0.2,
        pos_follow_tolerance: float = 0.2,
        pos_adjust_tolerance: float = 0.05,
        angle_adjust_tolerance: float = math.pi / 30,
        adjust_timeout: float = 8.0,
        recovery_timeout: float = 5.0,
        max_recovery_attempts: int = 3,
        recovery_behaviors: list[RecoveryBehavior] = [
            RecoveryBehavior.ROTATE,
            RecoveryBehavior.BACK,
            RecoveryBehavior.BACK_AND_ROTATE,
        ]
    ) -> None:
        self.state = State.IDLE
        self.target = None
        
        self._progress_thresh = progress_threshold
        self._max_npt = max_no_progress_time
        self._last_progress_ts = time.monotonic()
        self._last_progress_pos = (0, 0, 0)
        
        self._pos_follow_tol = pos_follow_tolerance
        self._pos_tol = pos_adjust_tolerance
        self._ang_tol = angle_adjust_tolerance
        
        self._rec_timeout = recovery_timeout
        self._max_rec_attempts = max_recovery_attempts
        
        self._rec_attempts = 0
        self._rec_start_ts = 0.0
        self._rec_behaviors = recovery_behaviors
        
        self._adjust_timeout = adjust_timeout
        self._adjust_start_ts = 0.0
    
    @classmethod
    def _dist2(cls, a, b):
        ax, ay = a[:2]
        bx, by = b[:2]
        return (ax - bx) ** 2 + (ay - by) ** 2
    
    @classmethod
    def _normalize_angle(cls, a):
        return (a + math.pi) % (2 * math.pi) - math.pi
    
    @classmethod
    def _dist(cls, a, b):
        return cls._dist2(a, b) ** 0.5
    
    @classmethod
    def _pose_diff(cls, a, b):
        dist = cls._dist(a, b)
        angle = cls._normalize_angle(a[2] - b[2])
        return dist, angle
    
    def feed(self, pos: tuple):
        if self.state in (State.IDLE, State.FAILURE, State.MANUAL):
            return
        
        elif self.state == State.FOLLOW_PATH:
            t = time.monotonic()

            # follow initialization
            if self._last_progress_pos == (0, 0, 0):
                self._last_progress_pos = pos
                self._last_progress_ts = t
                return
            
            dist2_progress = self._dist2(pos, self._last_progress_pos)
            dist2_target = self._dist(pos, self.target) if self.target else float('inf')
            
            if dist2_progress >= self._progress_thresh ** 2:
                self._last_progress_ts = t
                self._last_progress_pos = pos
                self._rec_attempts = 0
            
            elif t - self._last_progress_ts > self._max_npt:
                self._last_progress_ts = t
                self._rec_start_ts = t
                self._last_progress_pos = pos
                self.state = State.RECOVERY
                
                l.debug("entered State.RECOVERY from State.FOLLOW_PATH: progress timeout")
            
            elif dist2_target <= self._pos_follow_tol ** 2:
                self.state = State.ADJUST
                self._adjust_start_ts = time.monotonic()
                
                l.debug("entered State.ADJUST from State.FOLLOW_PATH")
        
        elif self.state == State.ADJUST:
            if self.target is None:
                self.state = State.IDLE
                
                l.debug("entered State.IDLE from State.ADJUST: target is None")
                return
            
            d_pose, d_angle = self._pose_diff(pos, self.target)
            if d_pose < self._pos_tol and d_angle < self._ang_tol:
                self.state = State.IDLE
                self.target = None
                
                l.debug("entered State.IDLE from State.ADJUST: reached target")
                return
            
            if time.monotonic() - self._adjust_start_ts > self._adjust_timeout:
                self.fail()
                l.debug("entered State.RECOVERY from State.ADJUST: adjust timeout exceeded")
        
        elif self.state == State.RECOVERY:
            if time.monotonic() - self._rec_start_ts > self._rec_timeout:
                self._rec_attempts += 1
                if self._rec_attempts >= self._max_rec_attempts:
                    self.state = State.FAILURE
                    
                    l.debug("entered State.FAILURE from State.RECOVERY: recovery attempts exceeded")
                
                else:
                    self.state = State.FOLLOW_PATH
                    self._last_progress_ts = time.monotonic()
                    self._last_progress_pos = pos
                    
                    l.debug("entered State.FOLLOW_PATH from State.RECOVERY")
        
        elif self.state == State.FAILURE:
            pass
        
        elif self.state == State.MANUAL:
            pass
    
    def nav_to(self, target: tuple | None):
        self._rec_start_ts = 0.0
        
        if target is None:
            self.state = State.IDLE
            l.debug("entered State.IDLE from 'nav_to': target is None")

        else:
            self.target = target
            self.state = State.FOLLOW_PATH
            self._last_progress_ts = time.monotonic()
            self._last_progress_pos = (0, 0, 0) # will be updated in 'feed'
            self._rec_attempts = 0
            
            l.debug("entered State.FOLLOW_PATH from 'nav_to'")
    
    def fail(self):
        self._rec_start_ts = time.monotonic()
        self.state = State.RECOVERY
    
    def manual(self):
        self.state = State.MANUAL
        
        l.debug("entered State.MANUAL")

    # getters for public api
    def get_recovery_behavior(self):
        if self.state != State.RECOVERY:
            return None

        # cycle through every available behavior
        return self._rec_behaviors[self._rec_attempts % len(self._rec_behaviors)]

    def get_adjust_heading(self):
        if self.state != State.ADJUST:
            return None
        
        if self.target is None:
            return None
        
        return self.target[2]
