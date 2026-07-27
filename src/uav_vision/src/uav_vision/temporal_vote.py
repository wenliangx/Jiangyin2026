"""Temporal majority voting with fail-closed current-frame semantics."""

from collections import Counter, deque


class TemporalVoter:
    def __init__(self, window_size, min_votes, lost_frames):
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        if min_votes <= 0 or min_votes > window_size:
            raise ValueError("min_votes must be in [1, window_size]")
        if lost_frames <= 0:
            raise ValueError("lost_frames must be positive")

        self.window_size = int(window_size)
        self.min_votes = int(min_votes)
        self.lost_frames = int(lost_frames)
        self._window = deque(maxlen=self.window_size)
        self._consecutive_unknown = 0

    def reset(self):
        self._window.clear()
        self._consecutive_unknown = 0

    def update(self, label):
        if label is None:
            self._consecutive_unknown += 1
            self._window.append(None)
            if self._consecutive_unknown >= self.lost_frames:
                self.reset()
            return None

        self._consecutive_unknown = 0
        self._window.append(str(label))
        votes = Counter(item for item in self._window if item is not None)
        if not votes:
            return None

        ranked = votes.most_common()
        winner, count = ranked[0]
        tied = len(ranked) > 1 and ranked[1][1] == count
        if tied or count < self.min_votes or winner != label:
            return None
        return winner
