/**
 * User Management Modal
 *
 * "My Account" section — available to all users:
 *   - Change own username (returns new token, updates auth store)
 *   - Change own password
 *
 * "User Management" section — admin only:
 *   - List all users with role badges
 *   - Reset any user's password
 *   - Delete any user except the protected "admin" account
 *   - Create new users (username, password, role)
 *
 * Sign out button at the bottom.
 */
import { useState, useEffect, useCallback } from 'react'
import { Plus, Trash2, KeyRound, UserPen, ShieldCheck, User, LogOut } from 'lucide-react'
import { Modal } from './common/Modal'
import { useAuthStore } from '../store/authStore'
import {
  listUsers,
  createUser,
  deleteUser,
  adminSetPassword,
  changePassword,
  changeUsername,
  type UserEntry,
} from '../api/auth'

interface Props {
  open: boolean
  onClose: () => void
}

const inputCls =
  'w-full px-3 py-2 rounded-xl bg-white/10 border border-white/20 text-white placeholder:text-white/40 outline-none focus:border-white/50 focus:bg-white/15 transition-all text-sm'

function btn(color: string) {
  return `px-4 py-2 rounded-xl text-sm font-semibold transition-all active:scale-95 disabled:opacity-50 disabled:cursor-not-allowed ${color}`
}

export function UserManagementModal({ open, onClose }: Props) {
  const { token, username: me, role, setAuth, clearAuth } = useAuthStore()

  // ── User list (admin only) ─────────────────────────────────────────────────
  const [users, setUsers] = useState<UserEntry[]>([])
  const [listError, setListError] = useState<string | null>(null)

  const refreshUsers = useCallback(async () => {
    if (role !== 'admin' || !token) return
    try {
      setListError(null)
      setUsers(await listUsers(token))
    } catch (e: any) {
      setListError(e.message)
    }
  }, [role, token])

  useEffect(() => {
    if (open) refreshUsers()
  }, [open, refreshUsers])

  // ── Delete user ────────────────────────────────────────────────────────────
  const [deleting, setDeleting] = useState<string | null>(null)

  const handleDelete = async (username: string) => {
    if (!token || !confirm(`Delete user "${username}"?`)) return
    setDeleting(username)
    try {
      await deleteUser(token, username)
      await refreshUsers()
    } catch (e: any) {
      alert(e.message)
    } finally {
      setDeleting(null)
    }
  }

  // ── Admin reset password inline form ──────────────────────────────────────
  const [resetTarget, setResetTarget] = useState<string | null>(null)
  const [resetPw, setResetPw] = useState('')
  const [resetError, setResetError] = useState<string | null>(null)
  const [resetOk, setResetOk] = useState(false)

  const handleAdminReset = async () => {
    if (!token || !resetTarget || !resetPw) return
    try {
      setResetError(null)
      await adminSetPassword(token, resetTarget, resetPw)
      setResetTarget(null)
      setResetPw('')
      setResetOk(true)
      setTimeout(() => setResetOk(false), 3000)
    } catch (e: any) {
      setResetError(e.message)
    }
  }

  // ── Create user ────────────────────────────────────────────────────────────
  const [newUsername, setNewUsername] = useState('')
  const [newPassword, setNewPassword] = useState('')
  const [newRole, setNewRole] = useState<'user' | 'admin'>('user')
  const [createError, setCreateError] = useState<string | null>(null)
  const [creating, setCreating] = useState(false)

  const handleCreate = async () => {
    if (!token || !newUsername.trim() || !newPassword) return
    setCreating(true)
    setCreateError(null)
    try {
      await createUser(token, newUsername.trim(), newPassword, newRole)
      setNewUsername('')
      setNewPassword('')
      setNewRole('user')
      await refreshUsers()
    } catch (e: any) {
      setCreateError(e.message)
    } finally {
      setCreating(false)
    }
  }

  // ── Change own username ────────────────────────────────────────────────────
  const [newMyUsername, setNewMyUsername] = useState('')
  const [unError, setUnError] = useState<string | null>(null)
  const [unOk, setUnOk] = useState(false)

  const handleChangeUsername = async () => {
    if (!token || !newMyUsername.trim()) return
    try {
      setUnError(null)
      const res = await changeUsername(token, newMyUsername.trim())
      setAuth(res.token, res.username, res.role)
      setNewMyUsername('')
      setUnOk(true)
      setTimeout(() => setUnOk(false), 3000)
      if (role === 'admin') await refreshUsers()
    } catch (e: any) {
      setUnError(e.message)
    }
  }

  // ── Change own password ────────────────────────────────────────────────────
  const [oldPw, setOldPw] = useState('')
  const [pw1, setPw1] = useState('')
  const [pw2, setPw2] = useState('')
  const [pwError, setPwError] = useState<string | null>(null)
  const [pwOk, setPwOk] = useState(false)

  const handleChangePw = async () => {
    if (!token) return
    if (!pw1) { setPwError('New password cannot be empty'); return }
    if (pw1 !== pw2) { setPwError("Passwords don't match"); return }
    try {
      setPwError(null)
      await changePassword(token, oldPw, pw1)
      setOldPw(''); setPw1(''); setPw2('')
      setPwOk(true)
      setTimeout(() => setPwOk(false), 3000)
    } catch (e: any) {
      setPwError(e.message)
    }
  }

  // ─────────────────────────────────────────────────────────────────────────

  return (
    <Modal open={open} onClose={onClose} title="Account & Users" maxWidth="max-w-2xl">
      <div className="space-y-8">

        {/* ── My Account ──────────────────────────────────────────────────── */}
        <section>
          <h3 className="text-xs font-semibold text-white/50 uppercase tracking-widest mb-4 flex items-center gap-2">
            <User className="size-3.5" />
            My Account
            <span className="ml-auto font-normal normal-case text-white/40">
              Signed in as{' '}
              <span className="text-white/70 font-medium">{me}</span>
              <span className={`ml-2 px-1.5 py-0.5 rounded-full text-xs ${role === 'admin' ? 'bg-amber-500/20 text-amber-300' : 'bg-white/10 text-white/50'}`}>
                {role}
              </span>
            </span>
          </h3>

          {/* Change username */}
          <div className="mb-5">
            <p className="text-xs text-white/50 mb-2">Change username</p>
            <div className="flex gap-2">
              <input
                className={inputCls}
                placeholder={`New username (current: ${me})`}
                value={newMyUsername}
                onChange={(e) => setNewMyUsername(e.target.value)}
                onKeyDown={(e) => e.key === 'Enter' && handleChangeUsername()}
              />
              <button
                onClick={handleChangeUsername}
                disabled={!newMyUsername.trim()}
                className={btn('bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/40 text-white')}
                title="Change username"
              >
                <UserPen className="size-4" />
              </button>
            </div>
            {unError && <p className="text-rose-400 text-xs mt-1">{unError}</p>}
            {unOk && <p className="text-emerald-400 text-xs mt-1">Username updated — a new token has been issued.</p>}
          </div>

          {/* Change password */}
          <div>
            <p className="text-xs text-white/50 mb-2">Change password</p>
            <div className="space-y-2">
              <input
                type="password"
                className={inputCls}
                placeholder="Current password"
                value={oldPw}
                onChange={(e) => setOldPw(e.target.value)}
                autoComplete="current-password"
              />
              <input
                type="password"
                className={inputCls}
                placeholder="New password"
                value={pw1}
                onChange={(e) => setPw1(e.target.value)}
                autoComplete="new-password"
              />
              <input
                type="password"
                className={inputCls}
                placeholder="Confirm new password"
                value={pw2}
                onChange={(e) => setPw2(e.target.value)}
                autoComplete="new-password"
              />
              <button
                onClick={handleChangePw}
                disabled={!oldPw || !pw1 || !pw2}
                className={btn('bg-white/10 border border-white/20 hover:bg-white/20 text-white w-full')}
              >
                Update Password
              </button>
            </div>
            {pwError && <p className="text-rose-400 text-xs mt-1">{pwError}</p>}
            {pwOk && <p className="text-emerald-400 text-xs mt-1">Password changed successfully.</p>}
          </div>
        </section>

        {/* ── Admin: User Management ──────────────────────────────────────── */}
        {role === 'admin' && (
          <section>
            <h3 className="text-xs font-semibold text-white/50 uppercase tracking-widest mb-4 flex items-center gap-2">
              <ShieldCheck className="size-3.5" />
              User Management
            </h3>

            {listError && <p className="text-rose-400 text-xs mb-2">{listError}</p>}
            {resetOk && <p className="text-emerald-400 text-xs mb-2">Password reset successfully.</p>}

            {/* User list */}
            <div className="space-y-2 mb-5">
              {users.map((u) => (
                <div
                  key={u.username}
                  className="flex items-center gap-3 px-4 py-2.5 rounded-xl bg-white/5 border border-white/10"
                >
                  <span className="flex-1 text-sm text-white font-medium">{u.username}</span>
                  <span
                    className={`text-xs px-2 py-0.5 rounded-full ${
                      u.role === 'admin'
                        ? 'bg-amber-500/20 text-amber-300'
                        : 'bg-white/10 text-white/50'
                    }`}
                  >
                    {u.role}
                  </span>

                  {/* Reset password */}
                  <button
                    onClick={() => {
                      setResetTarget(u.username)
                      setResetPw('')
                      setResetError(null)
                    }}
                    className="p-1.5 rounded-lg bg-white/5 hover:bg-white/15 transition-all"
                    title="Reset password"
                  >
                    <KeyRound className="size-3.5 text-white/60" />
                  </button>

                  {/* Delete — disabled for protected "admin" account */}
                  {u.username !== 'admin' ? (
                    <button
                      onClick={() => handleDelete(u.username)}
                      disabled={deleting === u.username}
                      className="p-1.5 rounded-lg bg-rose-500/10 hover:bg-rose-500/30 transition-all disabled:opacity-50"
                      title="Delete user"
                    >
                      <Trash2 className="size-3.5 text-rose-400" />
                    </button>
                  ) : (
                    /* Placeholder to keep alignment */
                    <div className="size-7" />
                  )}
                </div>
              ))}
            </div>

            {/* Inline reset-password form */}
            {resetTarget && (
              <div className="mb-5 p-4 rounded-xl bg-amber-500/10 border border-amber-400/20">
                <p className="text-xs text-amber-300 mb-3">
                  Reset password for <strong>{resetTarget}</strong>
                </p>
                <div className="flex gap-2">
                  <input
                    type="password"
                    className={inputCls}
                    placeholder="New password"
                    value={resetPw}
                    onChange={(e) => setResetPw(e.target.value)}
                    onKeyDown={(e) => e.key === 'Enter' && handleAdminReset()}
                    autoFocus
                  />
                  <button
                    onClick={handleAdminReset}
                    disabled={!resetPw}
                    className={btn('bg-amber-500/30 border border-amber-400/50 hover:bg-amber-500/40 text-white')}
                  >
                    Set
                  </button>
                  <button
                    onClick={() => setResetTarget(null)}
                    className={btn('bg-white/10 border border-white/20 hover:bg-white/20 text-white')}
                  >
                    Cancel
                  </button>
                </div>
                {resetError && <p className="text-rose-400 text-xs mt-1">{resetError}</p>}
              </div>
            )}

            {/* Create user */}
            <div>
              <p className="text-xs text-white/50 mb-2">Create new user</p>
              <div className="flex gap-2 flex-wrap">
                <input
                  className={`${inputCls} flex-1 min-w-[120px]`}
                  placeholder="Username"
                  value={newUsername}
                  onChange={(e) => setNewUsername(e.target.value)}
                />
                <input
                  type="password"
                  className={`${inputCls} flex-1 min-w-[120px]`}
                  placeholder="Password"
                  value={newPassword}
                  onChange={(e) => setNewPassword(e.target.value)}
                />
                <select
                  value={newRole}
                  onChange={(e) => setNewRole(e.target.value as 'user' | 'admin')}
                  className="px-3 py-2 rounded-xl bg-white/10 border border-white/20 text-white text-sm outline-none focus:border-white/50"
                >
                  <option value="user" className="bg-slate-800">user</option>
                  <option value="admin" className="bg-slate-800">admin</option>
                </select>
                <button
                  onClick={handleCreate}
                  disabled={creating || !newUsername.trim() || !newPassword}
                  className={btn('bg-emerald-500/30 border border-emerald-400/50 hover:bg-emerald-500/40 text-white')}
                  title="Create user"
                >
                  <Plus className="size-4" />
                </button>
              </div>
              {createError && <p className="text-rose-400 text-xs mt-1">{createError}</p>}
            </div>
          </section>
        )}

        {/* ── Sign out ─────────────────────────────────────────────────────── */}
        <div className="pt-4 border-t border-white/10 flex justify-end">
          <button
            onClick={() => { clearAuth(); onClose() }}
            className={btn('bg-rose-500/20 border border-rose-400/40 hover:bg-rose-500/30 text-rose-300 flex items-center gap-2')}
          >
            <LogOut className="size-4" />
            Sign Out
          </button>
        </div>

      </div>
    </Modal>
  )
}
