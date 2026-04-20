from dataclasses import dataclass
from enum import Enum, auto


class RobotState(Enum):
    SEARCHING = auto()
    FOLLOWING = auto()
    TURNING = auto()


@dataclass
class ControlDecision:
    state: RobotState
    turn_direction: str = 'none'
    wall_side: str = 'left'


class MazeStateMachine:
    def __init__(
        self,
        wall_detect_threshold: float,
        front_block_threshold: float,
        turn_exit_front_threshold: float,
    ) -> None:
        self.wall_detect_threshold = wall_detect_threshold
        self.front_block_threshold = front_block_threshold
        self.turn_exit_front_threshold = turn_exit_front_threshold
        self._state = RobotState.SEARCHING
        self._turn_direction = 'none'
        self._wall_side = 'left'

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def turn_direction(self) -> str:
        return self._turn_direction

    @property
    def wall_side(self) -> str:
        return self._wall_side

    def update(self, front_distance: float, left_distance: float, right_distance: float) -> ControlDecision:
        left_wall_seen = left_distance < self.wall_detect_threshold
        right_wall_seen = right_distance < self.wall_detect_threshold
        front_blocked = front_distance < self.front_block_threshold
        left_open = left_distance > self.wall_detect_threshold
        right_open = right_distance > self.wall_detect_threshold

        if left_wall_seen and right_wall_seen:
            self._wall_side = 'left' if left_distance <= right_distance else 'right'
        elif left_wall_seen:
            self._wall_side = 'left'
        elif right_wall_seen:
            self._wall_side = 'right'

        if front_blocked:
            self._state = RobotState.TURNING
            if self._wall_side == 'left' and left_open:
                self._turn_direction = 'left'
            elif self._wall_side == 'right' and right_open:
                self._turn_direction = 'right'
            elif left_open:
                self._turn_direction = 'left'
            elif right_open:
                self._turn_direction = 'right'
            else:
                self._turn_direction = 'right' if self._wall_side == 'left' else 'left'
            return ControlDecision(self._state, self._turn_direction, self._wall_side)

        if self._state == RobotState.TURNING and front_distance > self.turn_exit_front_threshold:
            self._state = RobotState.FOLLOWING if (left_wall_seen or right_wall_seen) else RobotState.SEARCHING
            self._turn_direction = 'none'

        if self._state != RobotState.TURNING:
            self._state = RobotState.FOLLOWING if (left_wall_seen or right_wall_seen) else RobotState.SEARCHING
            self._turn_direction = 'none'

        return ControlDecision(self._state, self._turn_direction, self._wall_side)
