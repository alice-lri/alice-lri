#!/bin/bash
set -e

pip wheel . -w dist
auditwheel repair dist/accurate_ri*.whl
