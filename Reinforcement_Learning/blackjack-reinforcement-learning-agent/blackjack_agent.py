import random
from collections import defaultdict
import numpy as np


class BlackjackAgent:
    def __init__(self):
        self.actions = [0, 1]
        self.discount_factor: float = 1
        self.lr = 0.01
        self.q_values = defaultdict(lambda: np.zeros(len(self.actions)))

    def action(self, observation, exploit_only=False) -> int:
        if not exploit_only:
            return random.choice(self.actions)
        else:
            return int(np.argmax(self.q_values[observation]))

    def learn(
        self, action, observation, reward, terminated, next_observation
    ) -> float:
        
        "*********Update qvalue*********"
        future_q_value = (not terminated) * np.max(self.q_values[next_observation])
        temporal_difference = (reward + self.discount_factor * future_q_value - self.q_values[observation][action])
        
        self.q_values[observation][action] = (self.q_values[observation][action] + self.lr * temporal_difference)
        """Returns the current error"""
        error = temporal_difference
        return error
