/**
 * Global WS send singleton.
 * Registered once by useWebSocket on mount.
 * Components import wsSend() directly — no prop drilling.
 */
type SendFn = (cmd: string, data: Record<string, any>) => void

let _send: SendFn = () => { console.warn('[WS] send called before connection') }

export const wsSend: SendFn = (cmd, data) => _send(cmd, data)
export const registerSend = (fn: SendFn) => { _send = fn }
