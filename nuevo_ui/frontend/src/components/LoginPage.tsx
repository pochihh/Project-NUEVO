/**
 * Login Page
 * Full-screen glassmorphic login form, matching the main app aesthetic.
 */
import { useState, type FormEvent } from 'react'
import { LogIn, Lock, User } from 'lucide-react'
import screwLogo from '../assets/screw_logo.svg'
import { login } from '../api/auth'
import { useAuthStore } from '../store/authStore'

export function LoginPage() {
  const setAuth = useAuthStore((s) => s.setAuth)
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(false)

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault()
    setError(null)
    setLoading(true)
    try {
      const res = await login(username.trim(), password)
      setAuth(res.token, res.username, res.role)
    } catch (err: any) {
      setError(err.message ?? 'Login failed')
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="min-h-screen w-full relative overflow-hidden flex items-center justify-center">
      {/* Background */}
      <div
        className="absolute inset-0"
        style={{ background: 'linear-gradient(45deg, #292E49, #536976)' }}
      >
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-cyan-400/20 rounded-full mix-blend-multiply filter blur-3xl animate-blob" />
          <div className="absolute top-1/3 right-1/4 w-96 h-96 bg-purple-400/20 rounded-full mix-blend-multiply filter blur-3xl animate-blob animation-delay-2000" />
          <div className="absolute bottom-1/4 right-1/3 w-96 h-96 bg-pink-400/20 rounded-full mix-blend-multiply filter blur-3xl animate-blob animation-delay-4000" />
        </div>
      </div>

      {/* Login card */}
      <div className="relative z-10 w-full max-w-sm mx-4">
        <div className="relative rounded-3xl p-8 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-2xl">
          {/* Top shimmer */}
          <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent rounded-t-3xl" />

          {/* Logo + title */}
          <div className="flex flex-col items-center mb-8">
            <div className="size-16 rounded-2xl backdrop-blur-xl bg-gradient-to-br from-white/20 to-white/10 border border-white/30 shadow-lg flex items-center justify-center mb-4">
              <img src={screwLogo} className="size-9 invert" alt="logo" />
            </div>
            <h1 className="text-2xl font-bold text-white">NUEVO UI</h1>
            <p className="text-sm text-white/60 mt-1">Sign in to control your robot</p>
          </div>

          <form onSubmit={handleSubmit} className="space-y-4">
            {/* Username */}
            <div className="relative">
              <User className="absolute left-3 top-1/2 -translate-y-1/2 size-4 text-white/50 pointer-events-none" />
              <input
                type="text"
                placeholder="Username"
                value={username}
                onChange={(e) => setUsername(e.target.value)}
                required
                autoComplete="username"
                autoFocus
                className="w-full pl-10 pr-4 py-3 rounded-xl bg-white/10 border border-white/20 text-white placeholder:text-white/40 outline-none focus:border-white/50 focus:bg-white/15 transition-all text-sm"
              />
            </div>

            {/* Password */}
            <div className="relative">
              <Lock className="absolute left-3 top-1/2 -translate-y-1/2 size-4 text-white/50 pointer-events-none" />
              <input
                type="password"
                placeholder="Password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                autoComplete="current-password"
                className="w-full pl-10 pr-4 py-3 rounded-xl bg-white/10 border border-white/20 text-white placeholder:text-white/40 outline-none focus:border-white/50 focus:bg-white/15 transition-all text-sm"
              />
            </div>

            {/* Error message */}
            {error && (
              <p className="text-rose-400 text-xs text-center px-2">{error}</p>
            )}

            {/* Submit */}
            <button
              type="submit"
              disabled={loading}
              className="w-full flex items-center justify-center gap-2 py-3 rounded-xl bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/45 active:scale-95 transition-all text-white font-semibold text-sm disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <LogIn className="size-4" />
              {loading ? 'Signing in…' : 'Sign In'}
            </button>
          </form>
        </div>
      </div>

      <style>{`
        @keyframes blob {
          0%   { transform: translate(0px, 0px) scale(1); }
          33%  { transform: translate(30px, -50px) scale(1.1); }
          66%  { transform: translate(-20px, 20px) scale(0.9); }
          100% { transform: translate(0px, 0px) scale(1); }
        }
        .animate-blob { animation: blob 7s infinite; }
        .animation-delay-2000 { animation-delay: 2s; }
        .animation-delay-4000 { animation-delay: 4s; }
      `}</style>
    </div>
  )
}
