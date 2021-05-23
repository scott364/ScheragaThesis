from gym.envs.registration import register
 
register(
    id='real-ur5-v0',
    #entry_point='balance_bot.envs:BalancebotEnv',
    entry_point='Gym_RealUR5.envs:UR5_RL_0',
)
register(
    id='real-ur5-v1',
    #entry_point='balance_bot.envs:BalancebotEnv',
    entry_point='Gym_RealUR5.envs:UR5_RL_1',
)