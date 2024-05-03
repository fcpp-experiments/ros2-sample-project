#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 \"<goal_info>\" (ex: GOAL-123456789)"
    exit 1
fi

timestamp=$(date +%s%3N)

goal_id="$1"
abort_file="from_user/goals/abort_$timestamp.txt"

echo "ABORT;$goal_id;0.0;0.0;0.0;0.0;0.0;0.0;" > "$abort_file"

echo "File created $abort_file, with goal_id: $goal_id"