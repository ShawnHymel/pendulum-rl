"""
An example that shows how to use SampleFactory to evaluate a trained agent.

python sf_eval.py --algo=APPO --experiment=my-pendulum --env=Pendulum-v1 --max_num_frames=200
"""

import sys

from sample_factory.cfg.arguments import parse_full_cfg, parse_sf_args
from sample_factory.envs.env_utils import register_env
from sample_factory.enjoy import enjoy

from env_gym import make_env

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
    parser, cfg = parse_sf_args(argv=None, evaluation=True)

    # Create config
    cfg = parse_full_cfg(parser, argv=None)

    # Run and evaluate the agent in the environment
    status = enjoy(cfg)

    return status
    
if __name__ == "__main__":
    sys.exit(main())