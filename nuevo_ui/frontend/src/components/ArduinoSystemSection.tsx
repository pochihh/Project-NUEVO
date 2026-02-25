import { Activity, AlertTriangle, CheckCircle, Play, Square, RotateCcw } from 'lucide-react';
import { useRobotStore } from '../store/robotStore';
import { wsSend } from '../lib/wsSend';

const STATE_LABELS: Record<number, { label: string; color: string }> = {
  0: { label: 'INIT',    color: 'text-white/60' },
  1: { label: 'IDLE',    color: 'text-amber-400' },
  2: { label: 'RUNNING', color: 'text-emerald-400' },
  3: { label: 'ERROR',   color: 'text-rose-400' },
  4: { label: 'E-STOP',  color: 'text-rose-500' },
};

function formatUptime(ms: number) {
  const s = Math.floor(ms / 1000);
  const m = Math.floor(s / 60);
  const h = Math.floor(m / 60);
  if (h > 0) return `${h}h ${m % 60}m`;
  if (m > 0) return `${m}m ${s % 60}s`;
  return `${s}s`;
}

export function ArduinoSystemSection() {
  const system        = useRobotStore((s) => s.system);
  const serialConnected = useRobotStore((s) => s.serialConnected);

  const firmwareState = system?.state ?? -1;
  const isRunning     = firmwareState === 2;
  const isEstop       = firmwareState === 4;

  const handleStart = () => wsSend('sys_cmd', { command: 1 });
  const handleStop  = () => wsSend('sys_cmd', { command: 2 });
  const handleReset = () => wsSend('sys_cmd', { command: 3 });

  const stateInfo = system ? (STATE_LABELS[system.state] ?? { label: 'UNKNOWN', color: 'text-white/60' }) : null;

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

      <div className="relative">
        {/* Title row + control buttons */}
        <div className="flex items-center justify-between mb-1">
          <h3 className="text-base font-semibold text-white">Arduino</h3>
          {serialConnected && (
            <div className="flex items-center gap-1.5">
              {isEstop ? (
                <button
                  onClick={handleReset}
                  className="flex items-center gap-1 px-2 py-1 rounded-lg bg-amber-500/40 border border-amber-400/60 hover:bg-amber-500/60 transition-all text-xs font-bold text-white animate-pulse"
                >
                  <RotateCcw className="size-3" />
                  Reset
                </button>
              ) : (
                <>
                  {!isRunning ? (
                    <button
                      onClick={handleStart}
                      className="flex items-center gap-1 px-2 py-1 rounded-lg bg-emerald-500/30 border border-emerald-400/50 hover:bg-emerald-500/40 transition-all text-xs font-semibold text-white"
                    >
                      <Play className="size-3" />
                      Start
                    </button>
                  ) : (
                    <button
                      onClick={handleStop}
                      className="flex items-center gap-1 px-2 py-1 rounded-lg bg-white/10 border border-white/20 hover:bg-white/20 transition-all text-xs font-semibold text-white"
                    >
                      <Square className="size-3" />
                      Stop
                    </button>
                  )}
                </>
              )}
            </div>
          )}
        </div>
        <p className="text-xs text-white/50 mb-4">System Status</p>

        <div className="space-y-3">
          {/* Firmware version + state */}
          {system ? (
            <>
              <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-2 space-y-1">
                <div className="flex items-center justify-between">
                  <span className="text-xs text-white/60">Firmware</span>
                  <span className="text-xs font-mono text-white">
                    v{system.firmwareMajor}.{system.firmwareMinor}.{system.firmwarePatch}
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-xs text-white/60">State</span>
                  <span className={`text-xs font-mono font-semibold ${stateInfo?.color}`}>
                    {stateInfo?.label}
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-xs text-white/60">Uptime</span>
                  <span className="text-xs font-mono text-white">{formatUptime(system.uptimeMs)}</span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-xs text-white/60">Free SRAM</span>
                  <span className="text-xs font-mono text-white">{system.freeSram} B</span>
                </div>
              </div>

              {/* Loop timing */}
              <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-2">
                <div className="text-xs text-white/70 mb-2 flex items-center gap-2">
                  <Activity className="size-3 text-cyan-400" />
                  <span>Loop Timing</span>
                </div>
                <div className="space-y-1">
                  <div className="flex items-center justify-between">
                    <span className="text-xs text-white/60">Avg</span>
                    <span className="text-xs font-mono text-white">{system.loopTimeAvgUs} µs</span>
                  </div>
                  <div className="flex items-center justify-between">
                    <span className="text-xs text-white/60">Max</span>
                    <span className="text-xs font-mono text-white">{system.loopTimeMaxUs} µs</span>
                  </div>
                </div>
              </div>

              {/* Errors */}
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <AlertTriangle className={`size-4 ${system.uartRxErrors > 0 ? 'text-amber-400' : 'text-emerald-400'}`} />
                  <span className="text-xs text-white/70">UART Errors</span>
                </div>
                <span className={`text-xs font-mono font-semibold ${system.uartRxErrors > 0 ? 'text-amber-400' : 'text-emerald-400'}`}>
                  {system.uartRxErrors}
                </span>
              </div>

              {/* Status indicator */}
              <div className="flex items-center gap-2 pt-2 border-t border-white/10">
                {system.state === 2 ? (
                  <>
                    <CheckCircle className="size-3 text-emerald-400" />
                    <span className="text-xs text-white/60">System Normal</span>
                  </>
                ) : system.state === 4 ? (
                  <>
                    <AlertTriangle className="size-3 text-rose-500" />
                    <span className="text-xs text-rose-400">E-STOP Active</span>
                  </>
                ) : system.state === 3 ? (
                  <>
                    <AlertTriangle className="size-3 text-rose-400" />
                    <span className="text-xs text-rose-400">Error (flags: 0x{system.errorFlags.toString(16)})</span>
                  </>
                ) : (
                  <>
                    <div className="size-2 rounded-full bg-amber-400 shadow-lg shadow-amber-400/50"></div>
                    <span className="text-xs text-white/60">{stateInfo?.label}</span>
                  </>
                )}
              </div>
            </>
          ) : (
            <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-2">
              <span className="text-xs text-white/40">Waiting for data...</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
