#!/bin/bash
set -eo pipefail
pushd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null

CONDA_ENV_NAME="accurate_ri_env"

mkdir -p .cache
module load cesga/system miniconda3/22.11.1-1

if ! command -v conda &> /dev/null
then
    echo "Conda could not be found"
    exit
fi

if ! conda env list | awk '{print $1}' | grep -wq "${CONDA_ENV_NAME}"; then
    echo "Conda environment \`${CONDA_ENV_NAME}\` does not exist. Creating..."
    conda create --name "${CONDA_ENV_NAME}" -y
    rm -f .cache/conda_env
fi

if [ conda_env.yml -nt .cache/conda_env ]; then
  echo "Updating conda environment..."
  conda env update --file conda_env.yml --name "${CONDA_ENV_NAME}" --prune
  touch .cache/conda_env
else
  echo "Conda environment is up to date."
fi

conda activate "${CONDA_ENV_NAME}"

popd > /dev/null