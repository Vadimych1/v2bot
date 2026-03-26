#!/bin/bash

cd pyrplidarsdk || exit 1

rm -rf dist
rm -rf build

PYTHON=${1:-python3}
echo "PythonPath: $PYTHON"

$PYTHON -m build --wheel

target_folder="dist"

first_file=$(find "$target_folder" -maxdepth 1 -type f -name "*.whl" | head -n 1)

if [ -n "$first_file" ] && [ -f "$first_file" ]; then
    $PYTHON -m pip install --force-reinstall --break-system-packages "$first_file"
else
    echo "build not found"
    exit 1
fi