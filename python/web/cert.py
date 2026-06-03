"""Self-signed cert helper for the local HTTPS WebXR server.

WebXR requires a secure context; the simplest path for a single-operator
LAN setup is a self-signed cert that the Quest browser accepts once.

This module:
  * Looks for an existing cert at ~/.aizee/quest_cert.pem (+ quest_key.pem)
  * Generates one if missing, with SANs for localhost + every local IPv4
  * Prints the SHA-256 fingerprint so the operator can verify on the Quest

Uses the `cryptography` library (pure-pip; works on Windows without
admin / openssl).
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import ipaddress
import socket
from pathlib import Path
from typing import Optional

try:
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID
    _CRYPTO_AVAILABLE = True
except ImportError:
    _CRYPTO_AVAILABLE = False


def _default_cert_dir() -> Path:
    return Path.home() / ".aizee"


def _local_ipv4s() -> list[str]:
    """Best-effort enumeration of local IPv4 addresses for SAN entries."""
    addrs: set[str] = {"127.0.0.1"}
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, family=socket.AF_INET):
            addrs.add(info[4][0])
    except Exception:
        pass
    # Also the route-out address (works without DNS).
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            addrs.add(s.getsockname()[0])
    except Exception:
        pass
    return sorted(addrs)


def ensure_self_signed(
    cert_dir: Optional[Path] = None,
    *,
    common_name: str = "aizee-quest-teleop",
    days_valid: int = 365 * 5,
) -> tuple[Path, Path, str]:
    """Return (cert_path, key_path, sha256_fingerprint).

    Reuses an existing pair if both files are present; otherwise generates
    a fresh RSA-2048 self-signed cert with SAN entries for localhost +
    every detected local IPv4.
    """
    if not _CRYPTO_AVAILABLE:
        raise RuntimeError(
            "The `cryptography` package is required for HTTPS — "
            "install via: pip install cryptography"
        )
    d = cert_dir if cert_dir is not None else _default_cert_dir()
    d.mkdir(parents=True, exist_ok=True)
    cert_path = d / "quest_cert.pem"
    key_path = d / "quest_key.pem"

    if not (cert_path.exists() and key_path.exists()):
        key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        ips = _local_ipv4s()
        san_entries: list[x509.GeneralName] = [x509.DNSName("localhost")]
        for ip in ips:
            try:
                san_entries.append(x509.IPAddress(ipaddress.ip_address(ip)))
            except ValueError:
                continue
        subject = issuer = x509.Name([
            x509.NameAttribute(NameOID.COMMON_NAME, common_name),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "AIZEE"),
        ])
        now = _dt.datetime.now(_dt.timezone.utc)
        cert = (
            x509.CertificateBuilder()
            .subject_name(subject)
            .issuer_name(issuer)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - _dt.timedelta(days=1))
            .not_valid_after(now + _dt.timedelta(days=days_valid))
            .add_extension(x509.SubjectAlternativeName(san_entries), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .sign(key, hashes.SHA256())
        )
        cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
        key_path.write_bytes(
            key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=serialization.NoEncryption(),
            )
        )

    cert_bytes = cert_path.read_bytes()
    fp = hashlib.sha256(_pem_der(cert_bytes)).hexdigest()
    fp_colon = ":".join(fp[i:i + 2] for i in range(0, len(fp), 2)).upper()
    return cert_path, key_path, fp_colon


def _pem_der(pem: bytes) -> bytes:
    """Extract the DER body from a PEM block for fingerprinting."""
    import base64
    lines = [ln for ln in pem.decode().splitlines() if "-----" not in ln and ln.strip()]
    return base64.b64decode("".join(lines))
