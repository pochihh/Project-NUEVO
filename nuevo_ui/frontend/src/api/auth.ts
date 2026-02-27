/**
 * Auth API client — thin fetch wrappers for /auth/* REST endpoints.
 */

export interface AuthResponse {
  token: string
  username: string
  role: string
}

export interface UserEntry {
  username: string
  role: string
}

function authHeaders(token: string): Record<string, string> {
  return { Authorization: `Bearer ${token}`, 'Content-Type': 'application/json' }
}

async function unwrap(res: Response): Promise<any> {
  if (!res.ok) {
    const body = await res.json().catch(() => ({}))
    throw new Error(body.detail ?? `HTTP ${res.status}`)
  }
  return res.json()
}

// ─── Shared endpoints ─────────────────────────────────────────────────────────

export async function login(username: string, password: string): Promise<AuthResponse> {
  return unwrap(
    await fetch('/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ username, password }),
    }),
  )
}

export async function getMe(token: string): Promise<{ username: string; role: string }> {
  return unwrap(await fetch('/auth/me', { headers: authHeaders(token) }))
}

export async function changePassword(
  token: string,
  oldPassword: string,
  newPassword: string,
): Promise<void> {
  await unwrap(
    await fetch('/auth/change-password', {
      method: 'POST',
      headers: authHeaders(token),
      body: JSON.stringify({ old_password: oldPassword, new_password: newPassword }),
    }),
  )
}

export async function changeUsername(
  token: string,
  newUsername: string,
): Promise<AuthResponse> {
  return unwrap(
    await fetch('/auth/change-username', {
      method: 'POST',
      headers: authHeaders(token),
      body: JSON.stringify({ new_username: newUsername }),
    }),
  )
}

// ─── Admin endpoints ──────────────────────────────────────────────────────────

export async function listUsers(token: string): Promise<UserEntry[]> {
  return unwrap(await fetch('/auth/users', { headers: authHeaders(token) }))
}

export async function createUser(
  token: string,
  username: string,
  password: string,
  role: string = 'user',
): Promise<void> {
  await unwrap(
    await fetch('/auth/users', {
      method: 'POST',
      headers: authHeaders(token),
      body: JSON.stringify({ username, password, role }),
    }),
  )
}

export async function deleteUser(token: string, username: string): Promise<void> {
  await unwrap(
    await fetch(`/auth/users/${encodeURIComponent(username)}`, {
      method: 'DELETE',
      headers: authHeaders(token),
    }),
  )
}

export async function adminSetPassword(
  token: string,
  username: string,
  newPassword: string,
): Promise<void> {
  await unwrap(
    await fetch(`/auth/users/${encodeURIComponent(username)}/password`, {
      method: 'PUT',
      headers: authHeaders(token),
      body: JSON.stringify({ new_password: newPassword }),
    }),
  )
}
