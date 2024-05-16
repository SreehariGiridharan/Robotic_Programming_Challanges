from statistics import mean

import gymnasium as gym
from gymnasium.wrappers.record_episode_statistics import (
    RecordEpisodeStatistics,
)
from tqdm.auto import trange

from blackjack_agent import BlackjackAgent


class BlackjackTrainer:
    def __init__(self, agent: BlackjackAgent, episode_count_mult=10):
        # observation_space_size calculation
        # The usable ace is only shown for player_sum >= 12.
        # [player_sum, dealer_card, usable_ace]
        # [12..21, 1..10, 0..1] => 10 * 10 * 2
        # [ 4..11, 1..10, 0   ] =>  8 * 10 * 1
        # FIXME observation_space must be fixed, use manual numbers
        # https://github.com/Farama-Foundation/Gymnasium/issues/803
        # from functools import reduce
        # from operator import mul
        # self.observation_space_size = reduce(
        #    mul, [s.n for s in self.env.observation_space]
        # )
        self.observation_space_size = 10 * 10 * 2 + 8 * 10 * 1

        self.episode_count = episode_count_mult * self.observation_space_size
        self.training_errors = []
        self.env = RecordEpisodeStatistics(
            gym.make("Blackjack-v1"), deque_size=self.episode_count
        )
        self.agent = agent

    def coverage(self):
        return (
            len(self.agent.observation_to_action_to_value)
            / self.observation_space_size
        )

    def play_episode_and_learn(self):
        observation, _ = self.env.reset()
        terminated = False
        while not terminated:
            action = self.agent.action(observation)
            next_observation, reward, terminated, _, _ = self.env.step(action)
            error = self.agent.learn(
                action, observation, reward, terminated, next_observation
            )
            self.training_errors.append(error)
            observation = next_observation

    def train(self):
        for _ in trange(self.episode_count, leave=False):
            self.play_episode_and_learn()
        return self.reward_per_step()

    def plot(self):
        import matplotlib.pyplot as plt
        import numpy as np
        from scipy.signal import convolve

        # Moving mean should increase automatically with episode count
        moving_mean_len = self.episode_count // 100
        return_queue_moving_mean = convolve(
            np.array(self.env.return_queue).flatten(),
            np.ones(moving_mean_len),
            mode="valid",
        )
        errors_moving_mean = convolve(
            self.training_errors,
            np.ones(moving_mean_len),
            mode="valid",
        )

        fig, ax = plt.subplots(ncols=2)

        fig.supxlabel("episode count")
        ax[0].set_title("reward (moving mean)")
        ax[0].plot(return_queue_moving_mean)
        ax[1].set_title("error (moving mean)")
        ax[1].plot(errors_moving_mean)
        fig.show()

    def reward_per_episode(self) -> float:
        return mean(r[0] for r in self.env.return_queue)

    def reward_per_step(self) -> float:
        return sum(self.env.return_queue)[0] / sum(self.env.length_queue)[0]

    def win_ratio(self) -> float:
        return mean(1 if r[0] == 1 else 0 for r in self.env.return_queue)

    def play_episode_with_exploitation_only(self):
        """to measure the performance after training"""
        observation, _ = self.env.reset()
        terminated = False
        while not terminated:
            action = self.agent.action(observation, exploit_only=True)
            next_observation, _, terminated, _, _ = self.env.step(action)
            observation = next_observation

    def play_with_exploitation_only(self, episode_count=None):
        episode_count_ = (
            self.episode_count if episode_count is None else episode_count
        )
        for _ in trange(episode_count_, leave=False):
            self.play_episode_with_exploitation_only()
        return self.reward_per_step()
