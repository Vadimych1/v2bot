#!/bin/bash

cd breezyslam || exit 1

rm -rf dist
rm -rf build

PYTHON=${1:-python3}

$PYTHON -m build --wheel

target_folder="dist"

first_file=$(find "$target_folder" -maxdepth 1 -type f -name "*.whl" | head -n 1)

if [ -n "$first_file" ] && [ -f "$first_file" ]; then
    $PYTHON -m pip install --force-reinstall "$first_file"
else
    echo "build not found"
    exit 1
fi