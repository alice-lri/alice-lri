#!/bin/bash
set -e

pip wheel . -w dist
auditwheel repair dist/alice_lri*.whl
