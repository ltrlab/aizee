"""Who may use Helm when it serves more than this computer (Helm on Minerva, listening beyond loopback).

- **A password**, kept as a scrypt hash in an access file:
  `python -m robot_platform.helm.access set-password FILE` (it asks for the password).
- **A login** gives the browser a session cookie (HttpOnly, SameSite=Strict, Secure over HTTPS), good for 12 hours of
  use since it was last used.
- **One device in control:** only the device in control starts, moves or changes anything. Any logged-in device may
  take control, and the one losing it is told.
- **Stopping is never locked out:** any logged-in device may Disable (H) or Shut down (X) a running session or sweep.
- **Wrong passwords slow down:** each failure from an address doubles its wait, up to a minute.

Helm on loopback only (the operator PC) needs none of this and uses none of it.
"""

import argparse
import getpass
import hashlib
import json
import os
from pathlib import Path
import secrets
import sys
import threading
import time

KIND = 'helm_access'
COOKIE = 'helm_session'
SESSION_S = 12 * 3600.0
SCRYPT = {'n': 1 << 14, 'r': 8, 'p': 1}
MIN_PASSWORD = 8
MAX_WAIT_S = 60.0


def hash_password(password, salt=None, **params):
    params = dict(SCRYPT, **params)
    salt = salt if salt is not None else secrets.token_bytes(16)
    digest = hashlib.scrypt(password.encode(), salt=salt, n=params['n'], r=params['r'], p=params['p'], dklen=32)
    return dict(params, salt=salt.hex(), hash=digest.hex())


def save_password(path, password):
    if len(password) < MIN_PASSWORD:
        raise ValueError(f'A Helm password has at least {MIN_PASSWORD} characters')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        'kind': KIND,
        'schema': 1,
        'scrypt': hash_password(password),
        'updated_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }
    temporary = path.with_suffix('.tmp')
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(document, indent=1) + '\n')
    os.replace(temporary, path)


class Access:
    def __init__(self, path, *, clock=time.monotonic):
        document = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(document, dict) or document.get('kind') != KIND or document.get('schema') != 1:
            raise ValueError(f'{path} is not a Helm access file')
        self._scrypt, self.clock = document['scrypt'], clock
        self._lock = threading.Lock()
        self._sessions = {}  # token -> {'device', 'address', 'seen'}
        self._failures = {}  # address -> (count, not_before)
        self._holder = None  # (token, since)

    # -- logging in -----------------------------------------------------------------------------

    def login(self, password, *, device, address):
        """A new session's token; a ValueError for a wrong password or one tried too soon."""
        now = self.clock()
        with self._lock:
            count, not_before = self._failures.get(address, (0, 0.0))
            if now < not_before:
                raise PermissionError(f'Too many wrong passwords: try again in {not_before - now:.0f} s')
        params = {key: self._scrypt[key] for key in ('n', 'r', 'p')}
        attempt = hash_password(str(password), bytes.fromhex(self._scrypt['salt']), **params)['hash']
        with self._lock:
            if not secrets.compare_digest(attempt, self._scrypt['hash']):
                count += 1
                self._failures[address] = (count, now + min(MAX_WAIT_S, 2.0 ** (count - 1)))
                raise ValueError('Wrong password')
            self._failures.pop(address, None)
            token = secrets.token_urlsafe(32)
            label = str(device or address or 'a device').strip()[:40] or 'a device'
            self._sessions[token] = {'device': label, 'address': address, 'seen': now}
            return token

    def session(self, token):
        """The session for `token` (refreshed), or None."""
        if not token:
            return None
        now = self.clock()
        with self._lock:
            session = self._sessions.get(token)
            if session is None or now - session['seen'] > SESSION_S:
                self._sessions.pop(token, None)
                if self._holder and self._holder[0] == token:
                    self._holder = None
                return None
            session['seen'] = now
            return dict(session)

    def logout(self, token):
        with self._lock:
            self._sessions.pop(token, None)
            if self._holder and self._holder[0] == token:
                self._holder = None

    # -- control ---------------------------------------------------------------------------------

    def in_control(self, token):
        with self._lock:
            return self._holder is not None and self._holder[0] == token and token in self._sessions

    def claim(self, token):
        """Take control (from whoever has it); the session must be logged in."""
        with self._lock:
            if token not in self._sessions:
                raise PermissionError('Log in first')
            self._holder = (token, self.clock())

    def release(self, token):
        with self._lock:
            if self._holder and self._holder[0] == token:
                self._holder = None

    def view(self, token):
        """What the page shows: this device, whether it is in control, and who is."""
        with self._lock:
            session = self._sessions.get(token)
            holder = None
            if self._holder is not None and self._holder[0] in self._sessions:
                holder = {
                    'device': self._sessions[self._holder[0]]['device'],
                    'since_s': round(self.clock() - self._holder[1], 1),
                }
            return {
                'enabled': True,
                'device': session['device'] if session else None,
                'in_control': bool(session) and self._holder is not None and self._holder[0] == token,
                'holder': holder,
            }


def stop_request(path, payload):
    """A request that only stops motion: Disable (H) or Shut down (X), for a teleop session or a sweep."""
    return (
        path in ('/api/helm/teleop/key', '/api/helm/tune/key')
        and isinstance(payload, dict)
        and payload.get('key') in ('H', 'X')
        and set(payload) == {'key'}
    )


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m robot_platform.helm.access')
    commands = parser.add_subparsers(dest='command', required=True)
    setting = commands.add_parser('set-password', help="Set Helm's password (it asks for it twice)")
    setting.add_argument('file', type=Path)
    args = parser.parse_args(argv)
    if sys.stdin.isatty():
        password = getpass.getpass('New Helm password: ')
        if getpass.getpass('Again: ') != password:
            print('The passwords differ; nothing changed.', file=sys.stderr)
            return 1
    else:
        password = sys.stdin.readline().rstrip('\n')
    save_password(args.file, password)
    print(f'Saved {args.file}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
