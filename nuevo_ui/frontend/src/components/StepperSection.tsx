import { useState } from 'react';
import { ChevronDown, Home } from 'lucide-react';
import { Switch } from './ui/switch';
import { Input } from './ui/input';
import { useRobotStore } from '../store/robotStore';
import { wsSend } from '../lib/wsSend';
import { Modal } from './common/Modal';

interface StepperSectionProps {
  stepperId: number;
}

const MOTION_STATES = ['Idle', 'Accel', 'Cruise', 'Decel', 'Homing', 'Fault'];

export function StepperSection({ stepperId }: StepperSectionProps) {
  const status = useRobotStore((s) => s.steppers[stepperId - 1]);
  const [showModal, setShowModal] = useState(false);
  const [maxVelocity, setMaxVelocity] = useState('1000');
  const [maxAcceleration, setMaxAcceleration] = useState('500');
  const [targetPosition, setTargetPosition] = useState('0');

  const isEnabled = (status?.enabled ?? 0) !== 0;
  const currentPosition = status?.commandedCount ?? 0;

  const handleEnable = (checked: boolean) => {
    wsSend('step_enable', { stepperNumber: stepperId, enable: checked ? 1 : 0 });
  };

  const handleApplyParams = () => {
    wsSend('step_set_params', {
      stepperNumber: stepperId,
      maxVelocity: parseFloat(maxVelocity),
      acceleration: parseFloat(maxAcceleration),
    });
  };

  const handleMove = () => {
    wsSend('step_move', {
      stepperNumber: stepperId,
      moveType: 0,  // 0 = absolute
      target: parseInt(targetPosition),
    });
  };

  const handleHome = () => {
    wsSend('step_home', { stepperNumber: stepperId });
  };

  return (
    <>
      <div
        onClick={() => setShowModal(true)}
        className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full cursor-pointer hover:bg-white/15 transition-all group"
      >
        <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
        <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

        <div className="relative">
          <div className="flex items-center justify-between mb-3">
            <h3 className="text-base font-semibold text-white">Stepper {stepperId}</h3>
            <ChevronDown className="size-4 text-white/70 group-hover:text-white transition-colors" />
          </div>

          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-2">
              <div className={`size-2 rounded-full ${isEnabled ? 'bg-emerald-400 shadow-lg shadow-emerald-400/50' : 'bg-white/30'}`}></div>
              <span className="text-xs text-white/60">{isEnabled ? 'Enabled' : 'Disabled'}</span>
            </div>
            <Switch
              checked={isEnabled}
              onCheckedChange={handleEnable}
              onClick={(e) => e.stopPropagation()}
            />
          </div>

          <div className="mt-3 pt-3 border-t border-white/10">
            <div className="text-sm text-white/50 text-left">Position: {currentPosition}</div>
          </div>
        </div>
      </div>

      {/* Modal */}
      <Modal
        open={showModal}
        onClose={() => setShowModal(false)}
        title={`Stepper ${stepperId}`}
        maxWidth="max-w-lg"
      >
        <div>
                {/* Live status */}
                <div className="rounded-2xl p-4 backdrop-blur-xl bg-white/5 border border-white/10 mb-6 space-y-2">
                  <div className="flex justify-between">
                    <span className="text-xs text-white/60">Position</span>
                    <span className="text-xs font-mono text-white">{status?.commandedCount ?? '--'} steps</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-xs text-white/60">Target</span>
                    <span className="text-xs font-mono text-white">{status?.targetCount ?? '--'} steps</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-xs text-white/60">Speed</span>
                    <span className="text-xs font-mono text-white">{status?.currentSpeed ?? '--'} steps/s</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-xs text-white/60">Motion</span>
                    <span className="text-xs font-mono text-white">
                      {status ? (MOTION_STATES[status.motionState] ?? 'Unknown') : '--'}
                    </span>
                  </div>
                  {(status?.limitHit ?? 0) !== 0 && (
                    <div className="flex justify-between">
                      <span className="text-xs text-amber-400">Limit Hit</span>
                      <span className="text-xs font-mono text-amber-400">
                        {(status!.limitHit & 1) ? 'Min ' : ''}{(status!.limitHit & 2) ? 'Max' : ''}
                      </span>
                    </div>
                  )}
                </div>

                {/* Motion Params */}
                <div className="grid grid-cols-2 gap-4 mb-4">
                  <div>
                    <label className="text-sm text-white/70 mb-2 block">Max Velocity (steps/s)</label>
                    <Input
                      type="number"
                      value={maxVelocity}
                      onChange={(e) => setMaxVelocity(e.target.value)}
                      className="backdrop-blur-xl bg-white/10 border-white/20 text-white"
                    />
                  </div>
                  <div>
                    <label className="text-sm text-white/70 mb-2 block">Max Acceleration</label>
                    <Input
                      type="number"
                      value={maxAcceleration}
                      onChange={(e) => setMaxAcceleration(e.target.value)}
                      className="backdrop-blur-xl bg-white/10 border-white/20 text-white"
                    />
                  </div>
                </div>
                <button
                  onClick={handleApplyParams}
                  className="w-full mb-4 px-4 py-2 rounded-xl backdrop-blur-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all text-sm font-semibold text-white"
                >
                  Apply Parameters
                </button>

                {/* Move */}
                <div className="mb-4">
                  <label className="text-sm text-white/70 mb-2 block">Target Position (steps)</label>
                  <div className="flex gap-3">
                    <Input
                      type="number"
                      value={targetPosition}
                      onChange={(e) => setTargetPosition(e.target.value)}
                      className="flex-1 backdrop-blur-xl bg-white/10 border-white/20 text-white"
                    />
                    <button
                      onClick={handleMove}
                      className="px-4 py-2 rounded-xl backdrop-blur-xl bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/40 transition-all text-sm font-semibold text-white"
                    >
                      Move
                    </button>
                  </div>
                </div>

                {/* Home */}
                <button
                  onClick={handleHome}
                  className="flex items-center gap-2 px-4 py-2 rounded-xl backdrop-blur-xl bg-amber-500/20 border border-amber-400/40 hover:bg-amber-500/30 transition-all text-sm font-semibold text-white"
                >
                  <Home className="size-4" />
                  Home
                </button>
        </div>
      </Modal>
    </>
  );
}
