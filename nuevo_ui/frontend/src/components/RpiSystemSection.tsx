import { Wifi, WifiOff, Cable, AlertCircle } from 'lucide-react';
import { useRobotStore } from '../store/robotStore';

export function RpiSystemSection() {
  const connection = useRobotStore((s) => s.connection);
  const connected  = useRobotStore((s) => s.connected);     // WS (browser → bridge)
  const serial     = useRobotStore((s) => s.serialConnected); // bridge ↔ Arduino

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

      <div className="relative">
        <h3 className="text-base font-semibold text-white mb-1">Raspberry Pi</h3>
        <p className="text-xs text-white/50 mb-4">Bridge Status</p>

        <div className="space-y-3">
          {/* WebSocket (browser → bridge) */}
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              {connected ? (
                <Wifi className="size-4 text-emerald-400" />
              ) : (
                <WifiOff className="size-4 text-rose-400" />
              )}
              <span className="text-xs text-white/70">WebSocket</span>
            </div>
            <span className={`text-xs font-semibold ${connected ? 'text-emerald-400' : 'text-rose-400'}`}>
              {connected ? 'Connected' : 'Disconnected'}
            </span>
          </div>

          {/* Arduino Serial */}
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              {serial ? (
                <Cable className="size-4 text-cyan-400" />
              ) : (
                <AlertCircle className="size-4 text-amber-400" />
              )}
              <span className="text-xs text-white/70">Arduino Serial</span>
            </div>
            <span className={`text-xs font-semibold ${serial ? 'text-cyan-400' : 'text-amber-400'}`}>
              {serial ? 'Connected' : 'Not found'}
            </span>
          </div>

          {/* Connection details */}
          {connection && (
            <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-2 space-y-1">
              {connection.port && (
                <div className="flex items-center justify-between">
                  <span className="text-xs text-white/60">Port</span>
                  <span className="text-xs font-mono text-white truncate max-w-[120px]" title={connection.port}>
                    {connection.port.split('/').pop()}
                  </span>
                </div>
              )}
              <div className="flex items-center justify-between">
                <span className="text-xs text-white/60">Baud</span>
                <span className="text-xs font-mono text-white">{connection.baud.toLocaleString()}</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-xs text-white/60">RX / TX</span>
                <span className="text-xs font-mono text-white">
                  {connection.rxCount} / {connection.txCount}
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-xs text-white/60">CRC Errors</span>
                <span className={`text-xs font-mono font-semibold ${connection.crcErrors > 0 ? 'text-amber-400' : 'text-white'}`}>
                  {connection.crcErrors}
                </span>
              </div>
            </div>
          )}

          {!connection && (
            <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-2">
              <span className="text-xs text-white/40">Waiting for bridge...</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
