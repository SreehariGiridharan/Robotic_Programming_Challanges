import argparse
from statistics import mean

from tqdm.auto import trange

from blackjack_agent import BlackjackAgent
from blackjack_trainer import BlackjackTrainer

parser = argparse.ArgumentParser()
parser.add_argument(
    "--run-count", "-n", default=10, help="Number of benchmark cycles"
)
run_count = parser.parse_args().run_count


def benchmark():
    trainer = BlackjackTrainer(BlackjackAgent())
    trainer.train()
    trainer.play_with_exploitation_only()
    return trainer.reward_per_step(), trainer.win_ratio()


# reward per step/s and win ratios
rpss, wrs = zip(
    *(benchmark() for _ in trange(run_count, desc="Benchmark run"))
)
print(
    "%.3f" % mean(rpss),
    "%.3f" % mean(wrs),
)
