#!/bin/bash
# Manage Basic Auth users for FLIR PTZ web dashboard.
# Usage:
#   ./manage_auth.sh add <username>      # add / update user
#   ./manage_auth.sh add <username> <password>  # non-interactive
#   ./manage_auth.sh del <username>      # delete user
#   ./manage_auth.sh list                # list users

HTPASSWD=/etc/nginx/.htpasswd

case "$1" in
  add)
    [ -z "$2" ] && echo "Usage: $0 add <username> [password]" && exit 1
    USER="$2"
    PASS="$3"

    if [ -n "$PASS" ]; then
      # Non-interactive: password provided as argument
      if [ -f "$HTPASSWD" ]; then
        echo "$PASS" | sudo htpasswd -i "$HTPASSWD" "$USER"
      else
        echo "$PASS" | sudo htpasswd -c -i "$HTPASSWD" "$USER"
      fi
    else
      # Interactive: prompt for password
      if [ -f "$HTPASSWD" ]; then
        sudo htpasswd "$HTPASSWD" "$USER"
      else
        sudo htpasswd -c "$HTPASSWD" "$USER"
      fi
    fi

    sudo systemctl reload nginx
    echo "✓ User '$USER' added/updated."
    ;;
  del)
    [ -z "$2" ] && echo "Usage: $0 del <username>" && exit 1
    USER="$2"
    read -p "Delete user '$USER'? [y/N] " confirm
    [[ "$confirm" != [yY] ]] && echo "Cancelled." && exit 0
    sudo htpasswd -D "$HTPASSWD" "$USER"
    sudo systemctl reload nginx
    echo "✓ User '$USER' deleted."
    ;;
  list)
    echo "Users in $HTPASSWD:"
    sudo cut -d: -f1 "$HTPASSWD"
    ;;
  *)
    echo "Usage: $0 {add|del|list} [username] [password]"
    exit 1
    ;;
esac
