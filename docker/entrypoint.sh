#!/bin/bash
# Container start: keep SSH host keys across rebuilds, hand the container env to SSH sessions, run sshd.
set -e

mkdir -p /etc/ssh/host_keys
for t in ed25519 rsa; do
    f=/etc/ssh/host_keys/ssh_host_${t}_key
    [ -f "$f" ] || ssh-keygen -q -t "$t" -N '' -f "$f"
done

# Sourced by /etc/bash.bashrc
export -p | grep -E ' (PATH|GZ_[A-Z_]*|ROS_[A-Z_]*|RMW_[A-Z_]*|CCACHE_[A-Z_]*|DISABLE_MAVNATIVE|PIP_[A-Z_]*|PYTHON[A-Z_]*)=' \
    > /etc/container.env

mkdir -p /run/sshd
exec /usr/sbin/sshd -D -e
