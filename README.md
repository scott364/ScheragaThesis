# ScheragaThesis

Files for Scott Scheraga's Master's Thesis work

env set up:

conda install python=3.6.9

conda create --name thesis python=3.6.9

conda activate thesis

conda install tensorflow

conda install jupyter

conda install jupyterlab

pip install gym baselines pybullet


go into cairo_simulator folder

pip install -e ./


go into baselines folder

pip install -e .

cd MainEnv_RL

pip install -e .


in bashrc, add
export PYTHONPATH="${PYTHONPATH}:/home/*USERNAMEHERE*/ScheragaThesis/MainEnv-RL"

https://backyardrobotics.wordpress.com/2017/11/27/build-a-balancing-bot-with-openai-gym-pt-i-setting-up/

https://backyardrobotics.wordpress.com/2017/11/29/build-a-balancing-bot-with-openai-gym-pt-ii-the-robot-and-environment/
