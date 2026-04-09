#!/usr/bin/env bash
#Run all scripts in the current directory.
set -e
#run test in the folder no matter which path  i am
cd "$(dirname "$0")"

# Run all scripts in the current directory except myself.
for script in *.sh; do
    if [[ "$script" != "build-all.sh" ]]; then
        echo "Running $script..."
        ./"$script"
    fi
done
