#!/bin/bash
# fix-copyright.sh — Add missing copyright headers to staged files.
# Used by the pre-commit hook to ensure all committed files have proper headers.

set -euo pipefail

GPL_HEADER_C='// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
'

GPL_HEADER_PY='# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
'

FIXED=0

for file in "$@"; do
  [ -f "$file" ] || continue

  case "$file" in
    *.cpp|*.hpp|*.h|*.cc|*.cxx|*.hxx)
      if ! head -1 "$file" | grep -q "^// Copyright"; then
        echo "$GPL_HEADER_C$(cat "$file")" > "$file"
        FIXED=$((FIXED + 1))
        echo "  Added copyright: $file"
      fi
      ;;
    *.py)
      # Skip if first non-shebang line already has copyright
      if ! head -5 "$file" | grep -q "^# Copyright"; then
        if head -1 "$file" | grep -q "^#!"; then
          # Preserve shebang
          SHEBANG=$(head -1 "$file")
          REST=$(tail -n +2 "$file")
          printf '%s\n%s\n%s' "$SHEBANG" "$GPL_HEADER_PY" "$REST" > "$file"
        else
          echo "$GPL_HEADER_PY$(cat "$file")" > "$file"
        fi
        FIXED=$((FIXED + 1))
        echo "  Added copyright: $file"
      fi
      ;;
  esac
done

if [ "$FIXED" -gt 0 ]; then
  echo "Fixed $FIXED file(s) — re-staging."
fi
