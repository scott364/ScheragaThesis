from gym.envs.registration import register
 
register(
    id='ur5-rl-v0',
    #entry_point='balance_bot.envs:BalancebotEnv',
    entry_point='UR5_RL.envs:UR5Env0',
)
