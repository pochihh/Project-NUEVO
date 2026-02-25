import { useState, useCallback, useEffect } from "react";
import { ChevronDown } from "lucide-react";
import { Switch } from "./ui/switch";
import { Input } from "./ui/input";
import { useRobotStore } from "../store/robotStore";
import { wsSend } from "../lib/wsSend";
import { DCPlot } from "./dc/DCPlot";
import { RecordCSV } from "./common/RecordCSV";
import { Modal } from "./common/Modal";

interface DCMotorSectionProps {
  motorId: number; // 1-based
}

type ControlMode = "velocity" | "position" | "pwm";

const MODES: ControlMode[] = ["velocity", "position", "pwm"];

// 1440 ticks/rev: encoder rated at 1440 CPR (4x quadrature), firmware uses ENCODER_4X
const TICKS_PER_REV = 1440;
// Max display speed: 2 rev/s = 100% on the bar
const MAX_REV_S = 2.0;

export function DCMotorSection({ motorId }: DCMotorSectionProps) {
  const motorState = useRobotStore((s) => s.dcMotors[motorId - 1]);
  const status     = motorState?.status ?? null;
  const setMotorRecording = useRobotStore((s) => s.setMotorRecording);

  const [showModal, setShowModal] = useState(false);
  const [controlMode, setControlMode] = useState<ControlMode>("velocity");
  const [targetPosition, setTargetPosition] = useState("1000");
  const [targetVelocity, setTargetVelocity] = useState("100");
  const [targetPwm, setTargetPwm] = useState("128");

  const [positionPID, setPositionPID] = useState({ p: "1.0", i: "0.1",  d: "0.05" });
  const [velocityPID, setVelocityPID] = useState({ p: "0.8", i: "0.05", d: "0.02" });

  // Sync PID from live status when modal opens
  useEffect(() => {
    if (showModal && status) {
      setPositionPID({
        p: status.posKp.toFixed(4),
        i: status.posKi.toFixed(4),
        d: status.posKd.toFixed(4),
      });
      setVelocityPID({
        p: status.velKp.toFixed(4),
        i: status.velKi.toFixed(4),
        d: status.velKd.toFixed(4),
      });
    }
  }, [showModal]); // intentionally omit status to avoid resetting while editing

  // ── Derived card display values ──────────────────────────────────────────
  const isEnabled   = (status?.mode ?? 0) !== 0;
  const pwmRaw      = status?.pwmOutput ?? 0;
  const dutyCycle   = (Math.abs(pwmRaw) / 255) * 100;
  const velAbs      = Math.abs(status?.velocity ?? 0);
  const speedRevs   = velAbs / TICKS_PER_REV;
  const speedPct    = Math.min(100, (speedRevs / MAX_REV_S) * 100);

  const modeIndex = MODES.indexOf(controlMode);

  // ── Commands ─────────────────────────────────────────────────────────────
  const handleEnable = (checked: boolean) => {
    const mode = checked ? (status?.mode || 2) : 0;
    wsSend('dc_enable', { motorNumber: motorId, mode });
  };

  // Mode numbers expected by firmware: 1=position, 2=velocity, 3=pwm
  const MODE_MAP: Record<ControlMode, number> = { position: 1, velocity: 2, pwm: 3 };

  const handleSendCommand = () => {
    const targetMode = MODE_MAP[controlMode];
    // If motor is enabled but in the wrong mode, switch it first
    if (isEnabled && status?.mode !== targetMode) {
      wsSend('dc_enable', { motorNumber: motorId, mode: targetMode });
    }
    if (controlMode === "velocity") {
      wsSend('dc_set_velocity', { motorNumber: motorId, targetTicks: parseInt(targetVelocity) });
    } else if (controlMode === "position") {
      wsSend('dc_set_position', { motorNumber: motorId, targetTicks: parseInt(targetPosition) });
    } else {
      wsSend('dc_set_pwm', { motorNumber: motorId, pwm: parseInt(targetPwm) });
    }
  };

  const handleApplyPID = () => {
    const pid = controlMode === "velocity" ? velocityPID : positionPID;
    wsSend('set_pid', {
      motorNumber: motorId,
      loopType: controlMode === "velocity" ? 1 : 0,
      kp: parseFloat(pid.p),
      ki: parseFloat(pid.i),
      kd: parseFloat(pid.d),
    });
  };

  // ── Recording ────────────────────────────────────────────────────────────
  const handleRecordingChange = useCallback((isRecording: boolean) => {
    setMotorRecording(motorId - 1, isRecording);
  }, [motorId, setMotorRecording]);

  // getData: returns only data from recordingStartTs onward (offset to 0)
  const getData = useCallback((): number[][] => {
    const ms = useRobotStore.getState().dcMotors[motorId - 1];
    const {
      timeHistory, positionHistory, velocityHistory,
      currentHistory, pwmHistory, frameIndexHistory, recordingStartTs,
    } = ms;

    // Find start index: first sample at or after recordingStartTs
    let startIdx = 0;
    if (recordingStartTs !== null) {
      const found = timeHistory.findIndex((t) => t >= recordingStartTs);
      startIdx = found >= 0 ? found : 0;
    }

    const n = timeHistory.length - startIdx;
    if (n === 0) return [[], [], [], [], [], []];

    const t0 = timeHistory[startIdx];
    const f0 = frameIndexHistory[startIdx] ?? 0;

    return [
      frameIndexHistory.slice(startIdx).map((f) => f - f0),           // Frame index (0-based)
      timeHistory.slice(startIdx).map((t) => (t - t0) / 1000),        // Time (s from 0)
      positionHistory.slice(startIdx),
      velocityHistory.slice(startIdx),
      currentHistory.slice(startIdx),
      pwmHistory.slice(startIdx),
    ];
  }, [motorId]);

  // ── Modal header: enable switch ──────────────────────────────────────────
  const enableAction = (
    <div className="flex items-center gap-2">
      <div className={`size-2 rounded-full ${isEnabled ? "bg-emerald-400 shadow-lg shadow-emerald-400/50" : "bg-white/30"}`} />
      <span className="text-xs text-white/60">{isEnabled ? "Enabled" : "Disabled"}</span>
      <Switch
        checked={isEnabled}
        onCheckedChange={handleEnable}
      />
    </div>
  );

  return (
    <>
      {/* ── Card ─────────────────────────────────────────────────────────── */}
      <div
        onClick={() => setShowModal(true)}
        className="relative rounded-2xl p-3 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl cursor-pointer hover:bg-white/15 transition-all group"
      >
        <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent" />
        <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />

        <div className="relative flex items-center">
          <div className="flex-1">
            <div className="flex items-center justify-between mb-2">
              <span className="text-base font-semibold text-white">M{motorId}</span>
              <div className="flex items-center gap-2 mr-3">
                <div className={`size-2 rounded-full ${isEnabled ? "bg-emerald-400 shadow-lg shadow-emerald-400/50" : "bg-white/30"}`} />
                <span className="text-xs text-white/60">{isEnabled ? "Enabled" : "Disabled"}</span>
                <Switch
                  checked={isEnabled}
                  onCheckedChange={handleEnable}
                  onClick={(e) => e.stopPropagation()}
                />
              </div>
            </div>

            {/* PWM bar */}
            <div className="flex items-center gap-2 mb-1">
              <span className="text-xs text-white/60 w-12">PWM</span>
              <div className="relative flex-1 h-2 rounded-full bg-white/10 border border-white/20 overflow-hidden">
                <div
                  className="absolute left-0 top-0 h-full bg-gradient-to-r from-cyan-400 to-blue-400 transition-all"
                  style={{
                    width: `${dutyCycle}%`,
                    boxShadow: isEnabled ? "0 0 6px rgba(34, 211, 238, 0.5)" : "none",
                    opacity: isEnabled ? 1 : 0.3,
                  }}
                />
              </div>
              <div className="flex items-baseline ml-2 w-20 text-xs">
                <span className="flex-1 text-right font-mono text-white/80">{dutyCycle.toFixed(0)}</span>
                <span className="w-10 text-left text-white/50 ml-1">%</span>
              </div>
            </div>

            {/* Speed bar */}
            <div className="flex items-center gap-2">
              <span className="text-xs text-white/60 w-12">Speed</span>
              <div className="relative flex-1 h-2 rounded-full bg-white/10 border border-white/20 overflow-hidden">
                <div
                  className="absolute left-0 top-0 h-full bg-gradient-to-r from-emerald-400 to-cyan-400 transition-all"
                  style={{
                    width: `${speedPct}%`,
                    boxShadow: isEnabled ? "0 0 6px rgba(16, 185, 129, 0.5)" : "none",
                    opacity: isEnabled ? 1 : 0.3,
                  }}
                />
              </div>
              <div className="flex items-baseline ml-2 w-20 text-xs">
                <span className="flex-1 text-right font-mono text-white/80">{speedRevs.toFixed(2)}</span>
                <span className="w-10 text-left text-white/50 ml-1">rev/s</span>
              </div>
            </div>
          </div>
          <ChevronDown className="size-4 text-white/70 group-hover:text-white transition-colors flex-shrink-0" />
        </div>
      </div>

      {/* ── Modal ────────────────────────────────────────────────────────── */}
      <Modal
        open={showModal}
        onClose={() => setShowModal(false)}
        title={`DC Motor ${motorId}`}
        subtitle={status ? `Pos: ${status.position} tks · Vel: ${status.velocity} tks/s · PWM: ${status.pwmOutput} · I: ${status.currentMa} mA` : undefined}
        headerActions={enableAction}
        maxWidth="max-w-4xl"
      >
        <div className="space-y-5">

          {/* ── Pill tab switcher ──────────────────────────────────────── */}
          <div className="relative flex rounded-full p-1 bg-white/10 border border-white/15">
            {/* Sliding glass pill */}
            <div
              className="absolute top-1 bottom-1 rounded-full bg-white/20 border border-white/30 shadow-inner transition-all duration-200 ease-in-out"
              style={{
                width: 'calc((100% - 8px) / 3)',
                left: `calc(4px + ${modeIndex} * (100% - 8px) / 3)`,
              }}
            />
            {MODES.map((mode) => (
              <button
                key={mode}
                onClick={() => setControlMode(mode)}
                className={`relative flex-1 py-2 text-sm font-semibold capitalize z-10 transition-colors duration-150 ${
                  controlMode === mode ? 'text-white' : 'text-white/40 hover:text-white/70'
                }`}
              >
                {mode}
              </button>
            ))}
          </div>

          {/* ── Target inputs ─────────────────────────────────────────── */}
          {controlMode === "pwm" ? (
            <div className="flex items-end gap-3">
              <div>
                <label className="text-xs text-white/60 mb-1.5 block">PWM output</label>
                <Input
                  type="number"
                  value={targetPwm}
                  onChange={(e) => setTargetPwm(e.target.value)}
                  placeholder="-255 to 255"
                  className="w-28 bg-white/10 border-white/20 text-white"
                />
              </div>
              <button
                onClick={handleSendCommand}
                className="px-5 py-2 rounded-xl bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/40 transition-all text-sm font-semibold text-white"
              >
                Send
              </button>
            </div>
          ) : (
            <div className="flex items-end gap-3">
              <div>
                <label className="text-xs text-white/60 mb-1.5 block">
                  {controlMode === "position" ? "Target (ticks)" : "Target (ticks/s)"}
                </label>
                <Input
                  type="number"
                  value={controlMode === "position" ? targetPosition : targetVelocity}
                  onChange={(e) => controlMode === "position"
                    ? setTargetPosition(e.target.value)
                    : setTargetVelocity(e.target.value)
                  }
                  className="w-32 bg-white/10 border-white/20 text-white"
                />
              </div>
              <div>
                <label className="text-xs text-white/60 mb-1.5 block">
                  {controlMode === "position" ? "Target (rev)" : "Target (rev/s)"}
                </label>
                <Input
                  type="number"
                  value={controlMode === "position"
                    ? (parseInt(targetPosition) / TICKS_PER_REV).toFixed(3)
                    : (parseInt(targetVelocity) / TICKS_PER_REV).toFixed(3)
                  }
                  onChange={(e) => {
                    const ticks = Math.round(parseFloat(e.target.value) * TICKS_PER_REV);
                    if (!isNaN(ticks)) {
                      controlMode === "position"
                        ? setTargetPosition(String(ticks))
                        : setTargetVelocity(String(ticks));
                    }
                  }}
                  className="w-28 bg-white/10 border-white/20 text-white"
                />
              </div>
              <button
                onClick={handleSendCommand}
                className="px-5 py-2 rounded-xl bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/40 transition-all text-sm font-semibold text-white"
              >
                Send
              </button>
            </div>
          )}

          {/* ── PID ──────────────────────────────────────────────────── */}
          {controlMode !== "pwm" && (
            <div>
              <span className="text-xs font-semibold text-white/50 block mb-2">
                {controlMode === "position" ? "Pos PID" : "Vel PID"}
              </span>
              <div className="flex items-center gap-2">
                {(["p", "i", "d"] as const).map((param) => (
                  <div key={param} className="flex items-center gap-1">
                    <span className="text-xs text-white/40 uppercase flex-shrink-0">{param}</span>
                    <Input
                      type="number"
                      step="0.001"
                      value={controlMode === "position" ? positionPID[param] : velocityPID[param]}
                      onChange={(e) => {
                        if (controlMode === "position") {
                          setPositionPID((prev) => ({ ...prev, [param]: e.target.value }));
                        } else {
                          setVelocityPID((prev) => ({ ...prev, [param]: e.target.value }));
                        }
                      }}
                      className="w-28 bg-white/10 border-white/20 text-white text-xs h-9 px-2"
                    />
                  </div>
                ))}
                <button
                  onClick={handleApplyPID}
                  className="flex-shrink-0 px-3 py-2 rounded-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all text-xs font-semibold text-white"
                >
                  Apply
                </button>
              </div>
            </div>
          )}

          {/* ── Recording ─────────────────────────────────────────────── */}
          <RecordCSV
            headers={["Frame Index", "Time (s)", "Position (ticks)", "Velocity (t/s)", "Current (mA)", "PWM"]}
            getData={getData}
            filename={`dc_motor_${motorId}`}
            onRecordingChange={handleRecordingChange}
          />

          {/* ── Live plot ─────────────────────────────────────────────── */}
          <div className="rounded-2xl p-4 bg-white/5 border border-white/10">
            <h3 className="text-sm font-semibold text-white mb-3">Real-time Data</h3>
            <DCPlot
              timeHistory={motorState?.timeHistory ?? []}
              positionHistory={motorState?.positionHistory ?? []}
              velocityHistory={motorState?.velocityHistory ?? []}
              currentHistory={motorState?.currentHistory ?? []}
              pwmHistory={motorState?.pwmHistory ?? []}
              recordingStartTs={motorState?.recordingStartTs ?? null}
            />
          </div>
        </div>
      </Modal>
    </>
  );
}
