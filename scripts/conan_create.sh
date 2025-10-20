#!/bin/bash
set -e

pushd "$(dirname "$0")" >/dev/null
pushd ../lib
conan create ../lib --user=samuel.soutullo --channel=stable
popd
popd
