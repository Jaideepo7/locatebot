#!/usr/bin/env bash
# Set up PostgreSQL for locatebot on the Raspberry Pi 5 (Raspberry Pi OS / Debian trixie). Idempotent.
#
#   sudo LOCATEBOT_DB_PASSWORD='<strong password>' deploy/postgres/setup_pi.sh <pi-ethernet-ip> <orin-ip-or-subnet>
#   e.g. ... setup_pi.sh 10.0.0.2 10.0.0.3/32
#
# - installs postgresql, creates role + database "locatebot" (password from the environment only,
#   never an argument, so it doesn't land in shell history or `ps`)
# - listens on localhost and the Pi's Ethernet address only (not Wi-Fi)
# - allows only the Orin (and local connections) to log in, with scram-sha-256
# - small-memory settings for the Pi (shared_buffers 64 MB, 20 connections)
# The Orin then uses: LOCATEBOT_DB_URL=postgresql://locatebot@<pi-ethernet-ip>/locatebot
# with the password in ~/.pgpass (chmod 600): <pi-ethernet-ip>:5432:locatebot:locatebot:<password>
set -euo pipefail

PI_IP="${1:?usage: setup_pi.sh <pi-ethernet-ip> <orin-ip-or-subnet>}"
ORIN_NET="${2:?usage: setup_pi.sh <pi-ethernet-ip> <orin-ip-or-subnet>}"
: "${LOCATEBOT_DB_PASSWORD:?set LOCATEBOT_DB_PASSWORD in the environment}"
[[ "$ORIN_NET" == */* ]] || ORIN_NET="$ORIN_NET/32"
[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }

if ! command -v pg_lsclusters >/dev/null 2>&1; then
  apt-get update
  apt-get install -y postgresql
fi
VER="$(pg_lsclusters --no-header | awk 'NR==1{print $1}')"
CONF_DIR="/etc/postgresql/${VER}/main"
[[ -d "$CONF_DIR" ]] || { echo "no cluster config in $CONF_DIR" >&2; exit 1; }

install -d -m 755 "$CONF_DIR/conf.d"
cat > "$CONF_DIR/conf.d/locatebot.conf" <<EOF
# locatebot (deploy/postgres/setup_pi.sh)
listen_addresses = 'localhost,${PI_IP}'
max_connections = 20
shared_buffers = 64MB
work_mem = 4MB
maintenance_work_mem = 32MB
password_encryption = 'scram-sha-256'
EOF

HBA="$CONF_DIR/pg_hba.conf"
LINE="host    locatebot    locatebot    ${ORIN_NET}    scram-sha-256"
grep -qxF "$LINE" "$HBA" || printf '%s\n' "# locatebot: the Orin only" "$LINE" >> "$HBA"

systemctl enable --now postgresql
systemctl restart postgresql

# Role and database; the password goes in through a psql variable, not the command line.
sudo -u postgres psql -v ON_ERROR_STOP=1 -v pw="$LOCATEBOT_DB_PASSWORD" <<'SQL'
SELECT format('CREATE ROLE locatebot LOGIN PASSWORD %L', :'pw')
 WHERE NOT EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'locatebot') \gexec
SELECT format('ALTER ROLE locatebot PASSWORD %L', :'pw') \gexec
SELECT 'CREATE DATABASE locatebot OWNER locatebot'
 WHERE NOT EXISTS (SELECT 1 FROM pg_database WHERE datname = 'locatebot') \gexec
SQL

echo "PostgreSQL ${VER} ready: postgresql://locatebot@${PI_IP}/locatebot (from ${ORIN_NET})."
echo "Tables are created by the robot on first connect (locatebot/storage/schema.py)."
