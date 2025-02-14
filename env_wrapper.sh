#!/bin/bash

source /home/samuel.soutullo/.miniconda3/etc/profile.d/conda.sh
conda activate accurate_ri_env
exec cmake "$@"
