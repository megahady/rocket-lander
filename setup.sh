#!/usr/bin/env bash
set -e

# Rocket Lander research platform setup script.
# Creates the Python virtual environment and installs dependencies.

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_ROOT"

if [ ! -d ".venv" ]; then
  python -m venv .venv
fi

# shellcheck source=/dev/null
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "Rocket Lander environment is ready. Activate it with: source .venv/bin/activate"
