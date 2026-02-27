/**
 * useWebSocket Hook
 * Manages WebSocket connection with auto-reconnect.
 * Dispatches incoming messages to the robot store.
 * Appends the JWT token as ?token= query param (required by the backend).
 */
import { useEffect, useRef, useCallback } from 'react'
import { useRobotStore } from '../store/robotStore'
import { useAuthStore } from '../store/authStore'
import type { WSMessage, WSCommand } from '../lib/wsProtocol'
import { registerSend } from '../lib/wsSend'

export function useWebSocket() {
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimerRef = useRef<number | null>(null)
  const isConnectingRef = useRef(false)
  const dispatch = useRobotStore((s) => s.dispatch)
  const token = useAuthStore((s) => s.token)

  const connect = useCallback(() => {
    if (isConnectingRef.current) return
    if (
      wsRef.current?.readyState === WebSocket.OPEN ||
      wsRef.current?.readyState === WebSocket.CONNECTING
    )
      return
    if (!token) return  // not authenticated — don't attempt connection

    isConnectingRef.current = true

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const wsUrl = `${protocol}//${window.location.host}/ws?token=${encodeURIComponent(token)}`

    console.log('[WS] Connecting to', wsUrl.split('?')[0])
    const socket = new WebSocket(wsUrl)

    socket.onopen = () => {
      console.log('[WS] Connected')
      useRobotStore.setState({ connected: true })
      isConnectingRef.current = false
    }

    socket.onmessage = (event) => {
      try {
        const message: WSMessage = JSON.parse(event.data)
        dispatch(message.topic, message.data, message.ts)
      } catch (err) {
        console.error('[WS] Failed to parse message:', err)
      }
    }

    socket.onclose = (event) => {
      console.log('[WS] Disconnected', event.code)
      useRobotStore.setState({ connected: false, serialConnected: false })
      wsRef.current = null
      isConnectingRef.current = false

      // code 4001 = auth rejected; don't retry
      if (event.code !== 1000 && event.code !== 4001 && !reconnectTimerRef.current) {
        reconnectTimerRef.current = window.setTimeout(() => {
          reconnectTimerRef.current = null
          connect()
        }, 2000)
      }
    }

    socket.onerror = () => {
      isConnectingRef.current = false
    }

    wsRef.current = socket
  }, [dispatch, token])

  useEffect(() => {
    connect()
    return () => {
      isConnectingRef.current = false
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current)
        reconnectTimerRef.current = null
      }
      if (wsRef.current) {
        wsRef.current.close(1000)
        wsRef.current = null
      }
    }
  }, [connect])

  const send = useCallback((cmd: string, data: Record<string, any>) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const command: WSCommand = { cmd, data }
      wsRef.current.send(JSON.stringify(command))
    } else {
      console.warn('[WS] Cannot send, not connected')
    }
  }, [])

  // Register with the global singleton so child components can send without prop drilling
  useEffect(() => { registerSend(send) }, [send])

  return { send }
}
