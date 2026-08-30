#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -eq 0 ]]; then
  echo "usage: $0 DEB..." >&2
  exit 2
fi

pending=()
for deb in "$@"; do
  package="$(dpkg-deb --field "${deb}" Package)"
  candidate="$(dpkg-deb --field "${deb}" Version)"
  installed="$(dpkg-query --showformat='${Version}' --show "${package}" 2>/dev/null || true)"

  if [[ -n "${installed}" ]] && dpkg --compare-versions "${installed}" ge "${candidate}"; then
    echo "keeping ${package} ${installed} (repository deb: ${candidate})"
    continue
  fi
  pending+=("${deb}")
done

if [[ "${#pending[@]}" -gt 0 ]]; then
  dpkg -i "${pending[@]}"
fi
