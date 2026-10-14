title: Open ruxd --local from a phone with a pairing code instead of a token URL
labels: gui, enhancement, backend, security

## Background
`rux gui` is gone; `ruxd --local` serves the web GUI (the ruxd-multiuser
design doc, phase S1, removed 2026-10-14 — see git history and
`docs/DIRECTION.md`'s 2026-10-08 changelog entries), and the multi-user
server (phase S3) has accounts, sessions and roles. What shipped for a phone on the LAN:
- `ruxd --local <dir> --bind <LAN-IP> --auth-token <token>`: a bind beyond
  loopback is refused without a token, and the token is then required on
  every request and on the events WebSocket.
- `http://<LAN-IP>:8420/?token=<token>` sets the per-port `HttpOnly`,
  `SameSite=Strict` cookie and redirects the token out of the URL; the page's
  own origin is then allowed for mutations, so `--allow-origin` is not needed.
- A Host check guards the loopback default against DNS rebinding.
- For a team, server mode with real accounts replaces all of this.

## Problem
The token still has to reach the phone by hand: typed, or the URL pasted into
a message. It is long-lived for the run, and the phone holds the same
all-powerful token as the desktop.

## Proposed fix
- [ ] `ruxd --local --lan`: bind the LAN address, generate the token, and
      print a short one-time pairing code plus a QR code of the URL in the
      terminal.
- [ ] The phone exchanges the code once for its own session cookie; the code
      expires after use or a few minutes.
- [ ] Keep loopback requests token-free, so the desktop flow is unchanged.
- [ ] An "Åbn på telefon" panel in the GUI showing the code and QR.

category=CLI estimate=3d
