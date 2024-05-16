import gymnasium as gym

env = gym.make("Blackjack-v1", render_mode="human")
observation, info = env.reset()
print("Enter 1 to hit, 0 to stick. Then press ↩️ .")
print("(observation), action, reward, terminated)")
try:
    while True:
        # action = env.action_space.sample()
        action = int(input("> "))
        observation, reward, terminated, truncated, info = env.step(action)
        print(observation, action, reward, terminated)

        if terminated or truncated:
            observation, info = env.reset()
            print(observation)
except KeyboardInterrupt:
    env.close()
