from gym.envs.registration import register
 
register(
    id='MainEnvRL-v0',
    #entry_point='balance_bot.envs:BalancebotEnv',
    entry_point='MainEnv_RL.envs:MainEnvRL',
)
