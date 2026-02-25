import { useState } from 'react';
import { ChevronDown } from 'lucide-react';
import { Switch } from './ui/switch';
import { Slider } from './ui/slider';
import { useRobotStore } from '../store/robotStore';
import { wsSend } from '../lib/wsSend';
import { Modal } from './common/Modal';

// Convert angle (0–180°) ↔ pulse width (500–2500 µs)
const angleToPulse = (angle: number) => Math.round(500 + (angle / 180) * 2000);
const pulseToAngle = (us: number) => Math.round(((us - 500) / 2000) * 180);

export function ServoSection() {
  const servo = useRobotStore((s) => s.servo);
  const [showModal, setShowModal] = useState(false);

  // Local angle state for the sliders (UI-driven, commands sent on change)
  const [angles, setAngles] = useState<number[]>(Array(16).fill(90));

  const getChannelEnabled = (ch: number) => {
    const item = servo?.channels.find((c) => c.channelNumber === ch);
    return item?.enabled ?? false;
  };

  const getChannelPulse = (ch: number) => {
    const item = servo?.channels.find((c) => c.channelNumber === ch);
    return item?.pulseUs ?? 1500;
  };

  const enabledCount = servo
    ? servo.channels.filter((c) => c.enabled).length
    : 0;

  const handleEnable = (ch: number, checked: boolean) => {
    wsSend('servo_enable', { channel: ch, enable: checked });
  };

  const handleAngle = (ch: number, angle: number) => {
    setAngles((prev) => {
      const next = [...prev];
      next[ch - 1] = angle;
      return next;
    });
    wsSend('servo_set', { channel: ch, pulseUs: angleToPulse(angle) });
  };

  // Sync angles from store when modal opens
  const handleOpenModal = () => {
    if (servo) {
      setAngles((prev) => {
        const next = [...prev];
        for (const ch of servo.channels) {
          if (ch.pulseUs > 0) next[ch.channelNumber - 1] = pulseToAngle(ch.pulseUs);
        }
        return next;
      });
    }
    setShowModal(true);
  };

  return (
    <>
      <div
        onClick={handleOpenModal}
        className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full cursor-pointer hover:bg-white/15 transition-all group"
      >
        <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
        <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

        <div className="relative">
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-semibold text-white">Servos</h3>
            <ChevronDown className="size-4 text-white/70 group-hover:text-white transition-colors" />
          </div>

          {/* Status indicators — 2 rows of 8 */}
          <div className="grid grid-cols-8 gap-1.5 mb-3">
            {Array.from({ length: 8 }, (_, i) => i + 1).map((ch) => (
              <div
                key={ch}
                className={`h-2 rounded-full ${
                  getChannelEnabled(ch)
                    ? 'bg-emerald-400 shadow-lg shadow-emerald-400/50'
                    : 'bg-white/20'
                } transition-all`}
              />
            ))}
          </div>
          <div className="grid grid-cols-8 gap-1.5 mb-3">
            {Array.from({ length: 8 }, (_, i) => i + 9).map((ch) => (
              <div
                key={ch}
                className={`h-2 rounded-full ${
                  getChannelEnabled(ch)
                    ? 'bg-emerald-400 shadow-lg shadow-emerald-400/50'
                    : 'bg-white/20'
                } transition-all`}
              />
            ))}
          </div>

          <div className="text-xs text-white/60 text-center">
            {servo ? `${enabledCount} / 16 enabled` : 'No PCA9685 data'}
          </div>
        </div>
      </div>

      {/* Modal */}
      <Modal
        open={showModal}
        onClose={() => setShowModal(false)}
        title="Servo Configuration"
        subtitle={servo ? `PCA9685 ${servo.pca9685Connected ? '● Connected' : '○ Not found'}${servo.pca9685Error !== 0 ? ` — error 0x${servo.pca9685Error.toString(16)}` : ''}` : undefined}
        maxWidth="max-w-5xl"
      >
        <div>
                {/* 4×4 grid */}
                <div className="grid grid-cols-4 gap-4">
                  {Array.from({ length: 16 }, (_, i) => i + 1).map((ch) => {
                    const enabled = getChannelEnabled(ch);
                    const angle   = angles[ch - 1];
                    const pulseUs = getChannelPulse(ch);
                    return (
                      <div
                        key={ch}
                        className="rounded-2xl p-4 backdrop-blur-xl bg-white/5 border border-white/10"
                      >
                        <div className="flex items-center justify-between mb-3">
                          <span className="text-sm font-semibold text-white">Ch {ch}</span>
                          <Switch
                            checked={enabled}
                            onCheckedChange={(checked) => handleEnable(ch, checked)}
                          />
                        </div>

                        <div className="mb-3">
                          <div className="flex items-center justify-between mb-2">
                            <span className="text-xs text-white/70">Angle</span>
                            <span className="text-sm font-mono text-white font-semibold">{angle}°</span>
                          </div>
                          <Slider
                            value={[angle]}
                            onValueChange={(val) => handleAngle(ch, val[0])}
                            min={0}
                            max={180}
                            step={1}
                            disabled={!enabled}
                            className="w-full"
                          />
                        </div>

                        <div className="flex items-center justify-between">
                          <div className={`size-2 rounded-full ${enabled ? 'bg-emerald-400 shadow-lg shadow-emerald-400/50' : 'bg-white/30'}`} />
                          <span className="text-xs font-mono text-white/50">{pulseUs} µs</span>
                        </div>
                      </div>
                    );
                  })}
                </div>
        </div>
      </Modal>
    </>
  );
}
