import { useState, useEffect, useRef } from 'react'

/**
 * Optimistic enable toggle for hardware switches.
 *
 * Usage:
 *   const { switchChecked, dotEnabled, pending, setOptimistic } = useOptimisticEnable(serverEnabled)
 *
 *   // On user toggle:
 *   setOptimistic(next)
 *   wsSend(...)
 *
 * - switchChecked  → bind to <Switch checked={...}>. Updates immediately on click.
 * - dotEnabled     → bind to the status dot indicator. Always shows real server state.
 * - pending        → true while waiting for server confirmation (switch and server disagree).
 * - setOptimistic  → call with the desired next state; auto-reverts after `timeoutMs` if
 *                    the server does not confirm the state change.
 */
export function useOptimisticEnable(serverEnabled: boolean, timeoutMs = 1500) {
  const [optimistic, setOptimisticState] = useState<boolean | null>(null)
  const timerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  // When the server state arrives matching our optimistic value → confirmed, clear it.
  useEffect(() => {
    if (optimistic !== null && serverEnabled === optimistic) {
      if (timerRef.current) { clearTimeout(timerRef.current); timerRef.current = null }
      setOptimisticState(null)
    }
    // intentionally omit `optimistic` — we only want to react to server state changes
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [serverEnabled])

  const setOptimistic = (next: boolean) => {
    setOptimisticState(next)
    if (timerRef.current) clearTimeout(timerRef.current)
    timerRef.current = setTimeout(() => {
      setOptimisticState(null) // revert — server did not confirm in time
      timerRef.current = null
    }, timeoutMs)
  }

  return {
    switchChecked: optimistic !== null ? optimistic : serverEnabled,
    dotEnabled: serverEnabled,
    pending: optimistic !== null,
    setOptimistic,
  }
}
