"""
An example that shows how to use SampleFactory with a Gym env.

sudo chmod 666 /dev/ttyACM0 

python sf_train.py --algo=APPO --use_rnn=False --num_workers=1 --num_envs_per_worker=1 --worker_num_splits=1 --policy_workers_per_policy=1 --recurrence=1 --with_vtrace=False --batch_size=512 --reward_scale=0.1 --save_every_sec=10 --experiment_summaries_interval=10 --experiment=pendulum-irl-01 --env=pendulum-irl --train_for_env_steps=300000

Note: to start over, delete the train_dir/ directory with:

rm -rf train_dir/

"""

import argparse
import sys

from sample_factory.cfg.arguments import parse_full_cfg, parse_sf_args
from sample_factory.envs.env_utils import register_env
from sample_factory.train import run_rl

from env_pendulum_irl import make_env


def main():

    # Find env argument
    env = None
    for i, arg in enumerate(sys.argv):
        if arg.startswith("--env" + '='):
            env = arg.split('=', 1)[1]
        elif arg == "--env" and i + 1 < len(sys.argv):
            env = sys.argv[i + 1]
            
    # Register environment 
    register_env(env, make_env)

    # If argv is None, then the following uses sys.argv[1:]
    parser, cfg = parse_sf_args(argv=None, evaluation=False)

    # Create config
    cfg = parse_full_cfg(parser, argv=None)

    # Train agent
    status = run_rl(cfg)
    
    return status

if __name__ == "__main__":
    sys.exit(main())