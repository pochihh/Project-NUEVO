/**
 * Auth Store (Zustand + localStorage persistence)
 *
 * Stores the JWT token, username, and role.
 * Token is persisted in localStorage so users stay logged in across
 * browser restarts. Token is invalidated on RPi reboot (new JWT secret).
 */
import { create } from 'zustand'
import { persist } from 'zustand/middleware'

export type Role = 'admin' | 'user'

interface AuthState {
  token: string | null
  username: string | null
  role: Role | null
  /** Store token + user info after a successful login or username change */
  setAuth: (token: string, username: string, role: string) => void
  /** Clear auth state (logout) */
  clearAuth: () => void
}

export const useAuthStore = create<AuthState>()(
  persist(
    (set) => ({
      token: null,
      username: null,
      role: null,
      setAuth: (token, username, role) =>
        set({ token, username, role: role as Role }),
      clearAuth: () => set({ token: null, username: null, role: null }),
    }),
    { name: 'nuevo-auth' },
  ),
)
