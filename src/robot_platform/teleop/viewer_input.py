"""Age-preserving read-only client of the existing local leader USB owner."""
from copy import deepcopy
import http.client
import json
import time
from urllib.parse import urlsplit

from .leader_bridge import LeaderSnapshotClient
from .openrb import LeaderSampleSkipped


def validate_viewer_url(url):
    """One local read-only source; no redirects, credentials, proxies or LAN URLs."""
    parsed = urlsplit(url)
    if (parsed.scheme != "http" or parsed.hostname != "127.0.0.1"
            or parsed.port != 8767 or parsed.username is not None
            or parsed.password is not None or parsed.path not in ("", "/")
            or parsed.query or parsed.fragment):
        raise ValueError("Leader viewer URL must be http://127.0.0.1:8767/")
    return parsed.hostname, parsed.port


class ViewerLeaderInput(LeaderSnapshotClient):
    """Persistent loopback HTTP connection using the existing nonce/age protocol."""
    def __init__(self, url, *, clock=time.perf_counter):
        host, port = validate_viewer_url(url)
        self.connection = http.client.HTTPConnection(host, port, timeout=.025)
        self.token = None
        try:
            profile = self._request("GET", "/api/profile")
            token = profile.get("action_token")
            if (not isinstance(token, str) or not token or len(token) > 256
                    or any(not 33 <= ord(char) <= 126 for char in token)):
                raise ValueError("Leader viewer did not supply a valid sampling token")
            if (not 0 < profile.get("max_age_s", .05) <= .05
                    or not 0 < profile.get("max_skew_s", .02) <= .02):
                raise ValueError("Shadow benchmark requires source age <=50 ms and skew <=20 ms")
            self.token = token
            super().__init__(self._exchange, profile, clock=clock)
            # Never persist the viewer action token in reports.
            self.profile = dict(id=profile.get("id"), leaders=deepcopy(profile["leaders"]),
                                max_age_s=self.max_age_s, max_skew_s=self.max_skew_s)
        except BaseException:
            self.close()
            raise

    def _request(self, method, path, payload=None):
        headers = {"Accept": "application/json"}
        body = None
        if payload is not None:
            body = json.dumps(payload, allow_nan=False).encode("utf-8")
            headers.update({"Content-Type": "application/json", "X-Leader-Token": self.token})
        self.connection.request(method, path, body=body, headers=headers)
        response = self.connection.getresponse()
        data = response.read(16385)
        if len(data) > 16384:
            raise ValueError(f"Leader viewer returned an oversized response ({response.status})")
        if response.status != 200:
            try:
                detail = json.loads(data)
            except ValueError:
                detail = {}
            if not isinstance(detail, dict):
                detail = {}
            error = str(detail.get("error") or "no detail")[:200]
            if response.status == 409 and detail.get("transient") is True:
                raise LeaderSampleSkipped(f"Leader viewer has no fresh sample: {error}")
            raise ValueError(f"Leader viewer rejected the request ({response.status}): {error}")
        result = json.loads(data)
        if not isinstance(result, dict):
            raise ValueError("Leader viewer response must be an object")
        return result

    def _exchange(self, request):
        try:
            return self._request("POST", "/api/sample", request)
        except LeaderSampleSkipped:
            raise  # Complete HTTP exchange; the connection remains usable.
        except BaseException:
            self.close()
            raise

    def close(self):
        self.connection.close()


