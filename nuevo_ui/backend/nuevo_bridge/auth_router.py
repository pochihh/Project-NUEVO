"""
Auth REST API Router for NUEVO Bridge.

Endpoints:
  POST   /auth/login                  — any user
  GET    /auth/me                     — any authenticated user
  POST   /auth/change-password        — change own password
  POST   /auth/change-username        — change own username (returns new token)
  GET    /auth/users                  — admin only: list all users
  POST   /auth/users                  — admin only: create user
  DELETE /auth/users/{username}       — admin only: delete user (not "admin")
  PUT    /auth/users/{username}/password — admin only: reset any password
"""
from fastapi import APIRouter, HTTPException, Header, Depends
from pydantic import BaseModel

from .auth import (
    load_users,
    save_users,
    verify_password,
    hash_password,
    create_token,
    decode_token,
    PROTECTED_USERNAMES,
)

router = APIRouter(prefix="/auth", tags=["auth"])


# ─── Dependency helpers ───────────────────────────────────────────────────────

def _bearer_token(authorization: str = Header(None)) -> str:
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing or malformed Authorization header")
    return authorization[7:]


def get_current_user(token: str = Depends(_bearer_token)) -> dict:
    return decode_token(token)


def require_admin(current: dict = Depends(get_current_user)) -> dict:
    if current.get("role") != "admin":
        raise HTTPException(status_code=403, detail="Admin access required")
    return current


# ─── Pydantic request models ──────────────────────────────────────────────────

class LoginRequest(BaseModel):
    username: str
    password: str


class ChangePasswordRequest(BaseModel):
    old_password: str
    new_password: str


class ChangeUsernameRequest(BaseModel):
    new_username: str


class AdminSetPasswordRequest(BaseModel):
    new_password: str


class CreateUserRequest(BaseModel):
    username: str
    password: str
    role: str = "user"


# ─── Shared endpoints ─────────────────────────────────────────────────────────

@router.post("/login")
def login(req: LoginRequest):
    """Authenticate and return a JWT token."""
    users = load_users()
    user = users.get(req.username)
    if not user or not verify_password(req.password, user["password_hash"]):
        raise HTTPException(status_code=401, detail="Invalid username or password")
    token = create_token(req.username, user["role"])
    return {"token": token, "username": req.username, "role": user["role"]}


@router.get("/me")
def me(current: dict = Depends(get_current_user)):
    """Validate token and return current user info."""
    users = load_users()
    if current["sub"] not in users:
        raise HTTPException(status_code=401, detail="Account no longer exists")
    return {"username": current["sub"], "role": current["role"]}


@router.post("/change-password")
def change_password(req: ChangePasswordRequest, current: dict = Depends(get_current_user)):
    """Change the calling user's own password."""
    users = load_users()
    username = current["sub"]
    if username not in users:
        raise HTTPException(status_code=404, detail="User not found")
    if not verify_password(req.old_password, users[username]["password_hash"]):
        raise HTTPException(status_code=401, detail="Current password is incorrect")
    users[username]["password_hash"] = hash_password(req.new_password)
    save_users(users)
    return {"ok": True}


@router.post("/change-username")
def change_username(req: ChangeUsernameRequest, current: dict = Depends(get_current_user)):
    """Change the calling user's own username. Returns a new token."""
    users = load_users()
    old = current["sub"]
    new = req.new_username.strip()

    if not new:
        raise HTTPException(status_code=400, detail="Username cannot be empty")
    if new == old:
        raise HTTPException(status_code=400, detail="New username is the same as current")
    if new in users:
        raise HTTPException(status_code=409, detail="Username already taken")

    users[new] = users.pop(old)
    save_users(users)
    token = create_token(new, users[new]["role"])
    return {"token": token, "username": new, "role": users[new]["role"]}


# ─── Admin-only endpoints ─────────────────────────────────────────────────────

@router.get("/users")
def list_users(current: dict = Depends(require_admin)):
    """List all users (admin only)."""
    users = load_users()
    return [{"username": u, "role": d["role"]} for u, d in users.items()]


@router.post("/users")
def create_user(req: CreateUserRequest, current: dict = Depends(require_admin)):
    """Create a new user (admin only)."""
    if req.role not in ("admin", "user"):
        raise HTTPException(status_code=400, detail="Role must be 'admin' or 'user'")
    users = load_users()
    if req.username in users:
        raise HTTPException(status_code=409, detail="Username already exists")
    users[req.username] = {
        "role": req.role,
        "password_hash": hash_password(req.password),
    }
    save_users(users)
    return {"ok": True}


@router.delete("/users/{username}")
def delete_user(username: str, current: dict = Depends(require_admin)):
    """Delete a user (admin only). The 'admin' account cannot be deleted."""
    if username in PROTECTED_USERNAMES:
        raise HTTPException(status_code=403, detail=f"Cannot delete protected account '{username}'")
    users = load_users()
    if username not in users:
        raise HTTPException(status_code=404, detail="User not found")
    del users[username]
    save_users(users)
    return {"ok": True}


@router.put("/users/{username}/password")
def admin_set_password(
    username: str,
    req: AdminSetPasswordRequest,
    current: dict = Depends(require_admin),
):
    """Reset any user's password (admin only)."""
    users = load_users()
    if username not in users:
        raise HTTPException(status_code=404, detail="User not found")
    users[username]["password_hash"] = hash_password(req.new_password)
    save_users(users)
    return {"ok": True}
