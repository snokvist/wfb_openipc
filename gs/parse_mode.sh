#!/bin/bash
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <preferred_modes_comma_separated>" >&2
  exit 1
fi

# The comma‑separated preference list, e.g. "1280x720@100,1280x720@70,1920x1080@144"
pref_str="$1"

# Split into an array on commas
IFS=',' read -r -a prefs <<< "$pref_str"

# Get only the mode lines (those starting with a digit)
modes=$(pixelpilot --screen-mode-list | grep -E '^[0-9]')

# Try each preference in order
for pref in "${prefs[@]}"; do
  if echo "$modes" | grep -Fxq "$pref"; then
    # Print the first match and exit
    printf '%s' "$pref"
    exit 0
  fi
done

# No preference matched: fall back to the first supported mode
fallback=$(printf '%s\n' "$modes" | head -n1)
printf '%s' "$fallback"
