import { useState } from 'react';
import { motion } from 'motion/react';
import { AlertTriangle, ChevronDown } from 'lucide-react';
import { useRobotStore } from '../store/robotStore';

type BatteryType = '10 Cell NiMH' | '3S LiPo' | '4S LiPo';

const BATTERY_RANGES: Record<BatteryType, { min: number; max: number }> = {
  '10 Cell NiMH': { min: 9, max: 13 },
  '3S LiPo':      { min: 9.9, max: 12.6 },
  '4S LiPo':      { min: 13.2, max: 16.8 },
};

export function PowerSection() {
  const voltage = useRobotStore((s) => s.voltage);
  const [batteryType, setBatteryType] = useState<BatteryType>('10 Cell NiMH');
  const [showBatteryDropdown, setShowBatteryDropdown] = useState(false);

  const batteryVoltage = voltage ? voltage.batteryMv / 1000 : 0;
  const servoVoltage   = voltage ? voltage.servoRailMv / 1000 : 0;

  const batteryRange = BATTERY_RANGES[batteryType];
  const servoRange = { min: 5, max: 10 };

  const getBatteryWarning = () => {
    if (!voltage) return { show: false, type: '' };
    if (batteryVoltage < batteryRange.min) return { show: true, type: 'low' };
    if (batteryVoltage > batteryRange.max) return { show: true, type: 'high' };
    return { show: false, type: '' };
  };

  const getServoWarning = () => {
    if (!voltage) return { show: false, type: '' };
    if (servoVoltage < 5.5) return { show: true, type: 'low' };
    if (servoVoltage > 9) return { show: true, type: 'high' };
    return { show: false, type: '' };
  };

  const batteryWarning = getBatteryWarning();
  const servoWarning = getServoWarning();

  const batteryPct = Math.max(0, Math.min(100,
    ((batteryVoltage - batteryRange.min) / (batteryRange.max - batteryRange.min)) * 100
  ));
  const servoPct = Math.max(0, Math.min(100,
    ((servoVoltage - servoRange.min) / (servoRange.max - servoRange.min)) * 100
  ));

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

      <div className="relative">
        <div className="flex items-center justify-between mb-3">
            <h3 className="text-lg font-semibold text-white">Power</h3>
            {/* Battery Type Selector */}
            <div className="relative">
              <button
                onClick={() => setShowBatteryDropdown(!showBatteryDropdown)}
                className="w-full px-3 py-1 rounded-xl backdrop-blur-xl bg-white/10 border border-white/20 text-white text-sm flex items-center justify-between hover:bg-white/15 transition-all"
              >
                <span>{batteryType}</span>
                <ChevronDown className="size-4" />
              </button>

              {showBatteryDropdown && (
                <motion.div
                  initial={{ opacity: 0, y: -10 }}
                  animate={{ opacity: 1, y: 0 }}
                  className="absolute z-50 w-full mt-2 rounded-xl backdrop-blur-2xl bg-white/20 border border-white/30 shadow-2xl overflow-hidden"
                >
                  {(['10 Cell NiMH', '3S LiPo', '4S LiPo'] as BatteryType[]).map((type) => (
                    <button
                      key={type}
                      onClick={() => {
                        setBatteryType(type);
                        setShowBatteryDropdown(false);
                      }}
                      className="w-full px-3 py-2 text-sm text-white hover:bg-white/20 transition-all text-left"
                    >
                      {type}
                    </button>
                  ))}
                </motion.div>
              )}
            </div>
        </div>

        {/* Voltage Section */}
        <div className="flex gap-4">

          {/* Battery Voltage */}
          <div className="flex-[5]">
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm text-white/70">Battery</span>
              <div className="flex items-center gap-2">
                <span className="text-sm font-mono text-white font-semibold">
                  {voltage ? batteryVoltage.toFixed(1) : '--'}V
                </span>
                {batteryWarning.show && (
                  <AlertTriangle className="size-4 text-amber-400" />
                )}
              </div>
            </div>

            <div className="h-3 rounded-full backdrop-blur-xl bg-white/10 border border-white/20 overflow-hidden">
              <div
                className={`h-full transition-all duration-300 ${
                  batteryWarning.show ? 'bg-amber-400' : 'bg-emerald-400'
                }`}
                style={{
                  width: `${batteryPct}%`,
                  boxShadow: batteryWarning.show
                    ? '0 0 12px rgba(251, 191, 36, 0.6)'
                    : '0 0 12px rgba(52, 211, 153, 0.6)',
                }}
              />
            </div>

            <div className="flex justify-between mt-1">
              <span className="text-xs text-white/50">{batteryRange.min}V</span>
              <span className="text-xs text-white/50">{batteryRange.max}V</span>
            </div>
          </div>

          {/* Servo Voltage */}
          <div className="flex-[2]">
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm text-white/70">Servo</span>
              <div className="flex items-center gap-2">
                <span className="text-sm font-mono text-white font-semibold">
                  {voltage ? servoVoltage.toFixed(1) : '--'}V
                </span>
                {servoWarning.show && (
                  <AlertTriangle className="size-4 text-amber-400" />
                )}
              </div>
            </div>

            <div className="h-3 rounded-full backdrop-blur-xl bg-white/10 border border-white/20 overflow-hidden">
              <div
                className={`h-full transition-all duration-300 ${
                  servoWarning.show ? 'bg-amber-400' : 'bg-cyan-400'
                }`}
                style={{
                  width: `${servoPct}%`,
                  boxShadow: servoWarning.show
                    ? '0 0 12px rgba(251, 191, 36, 0.6)'
                    : '0 0 12px rgba(34, 211, 238, 0.6)',
                }}
              />
            </div>

            <div className="flex justify-between mt-1">
              <span className="text-xs text-white/50">{servoRange.min}V</span>
              <span className="text-xs text-white/50">{servoRange.max}V</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
