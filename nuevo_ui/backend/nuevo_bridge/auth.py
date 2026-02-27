"""
Authentication helpers for NUEVO Bridge.

- Users stored in backend/users.json (ignored by git).
- Passwords hashed with bcrypt via passlib.
- JWT tokens signed with a random secret (per-process).
  Users must re-login after RPi reboot — this is intentional and acceptable.
- Default accounts (admin / user) are created automatically on first boot.
"""
import json
import secrets
from pathlib import Path

import bcrypt
import jwt
from datetime import datetime, timedelta, timezone
from fastapi import HTTPException

# ─── Paths ────────────────────────────────────────────────────────────────────
# users.json lives at backend/users.json (parent of the nuevo_bridge package dir)
USERS_FILE = Path(__file__).parent.parent / "users.json"

# ─── Password hashing (bcrypt directly — avoids passlib compatibility issues) ─

def verify_password(plain: str, hashed: str) -> bool:
    return bcrypt.checkpw(plain.encode("utf-8"), hashed.encode("utf-8"))


def hash_password(plain: str) -> str:
    return bcrypt.hashpw(plain.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")


# ─── JWT ──────────────────────────────────────────────────────────────────────
# Random secret per process start — users re-login after RPi reboot.
JWT_SECRET = secrets.token_hex(32)
JWT_ALGORITHM = "HS256"
JWT_EXPIRE_DAYS = 30


def create_token(username: str, role: str) -> str:
    payload = {
        "sub": username,
        "role": role,
        "exp": datetime.now(timezone.utc) + timedelta(days=JWT_EXPIRE_DAYS),
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def decode_token(token: str) -> dict:
    """Return JWT payload dict (keys: sub, role). Raises HTTPException on failure."""
    try:
        return jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired — please log in again")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")


# ─── Default users ────────────────────────────────────────────────────────────
# These credentials are committed to git; users.json (with hashed passwords)
# is NOT committed (see .gitignore). On first boot users.json is auto-created.
_DEFAULT_USERS_PLAIN = {
    "admin": {"role": "admin", "plain_password": "admin1540"},
    "user":  {"role": "user",  "plain_password": "162"},
}

# The "admin" username cannot be deleted (but CAN be renamed if desired).
PROTECTED_USERNAMES = {"admin"}  # usernames that cannot be deleted


def _create_default_users() -> dict:
    """Hash default passwords and return the users dict (called once on first boot)."""
    print("[Auth] Hashing default passwords — this may take a moment…")
    return {
        name: {
            "role": info["role"],
            "password_hash": hash_password(info["plain_password"]),
        }
        for name, info in _DEFAULT_USERS_PLAIN.items()
    }


# ─── User CRUD ────────────────────────────────────────────────────────────────
# Simple in-memory cache — avoids a disk read on every auth request.
# Cache is invalidated by save_users() so it always reflects the current state.

_users_cache = None  # type: dict | None  # populated on first load


def load_users() -> dict:
    """Load users from cache or disk. Creates users.json with defaults if missing."""
    global _users_cache
    if _users_cache is not None:
        return _users_cache
    if not USERS_FILE.exists():
        users = _create_default_users()
        save_users(users)   # also populates cache
        print(f"[Auth] Created default users.json at {USERS_FILE}")
        return _users_cache  # type: ignore[return-value]
    with open(USERS_FILE) as f:
        _users_cache = json.load(f)
    return _users_cache


def save_users(users: dict) -> None:
    global _users_cache
    USERS_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(USERS_FILE, "w") as f:
        json.dump(users, f, indent=2)
    _users_cache = users  # keep cache in sync
