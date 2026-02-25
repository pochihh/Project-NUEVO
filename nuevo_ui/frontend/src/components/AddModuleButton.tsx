import { useState } from 'react';
import { motion, AnimatePresence } from 'motion/react';
import { Plus, X, Camera, MapPin, Activity } from 'lucide-react';

type ModuleType = 'camera' | 'gps' | 'imu' | 'lidar' | 'ultrasonic';

interface Module {
  id: number;
  type: ModuleType;
  name: string;
}

interface AddModuleButtonProps {
  type: 'qwiic' | 'sensor';
  onAdd: (module: Module) => void;
}

const qwiicModules = [
  { type: 'camera' as ModuleType, name: 'Camera Feed', icon: Camera },
  { type: 'gps' as ModuleType, name: 'GPS Location', icon: MapPin },
  { type: 'imu' as ModuleType, name: 'IMU Data', icon: Activity },
];

const sensorModules = [
  { type: 'imu' as ModuleType, name: 'IMU Visualization', icon: Activity },
  { type: 'lidar' as ModuleType, name: 'Lidar Data', icon: Activity },
  { type: 'ultrasonic' as ModuleType, name: 'Ultrasonic Sensor', icon: Activity },
];

export function AddModuleButton({ type, onAdd }: AddModuleButtonProps) {
  const [showMenu, setShowMenu] = useState(false);
  const modules = type === 'qwiic' ? qwiicModules : sensorModules;

  const handleAdd = (moduleType: ModuleType, name: string) => {
    onAdd({
      id: Date.now(),
      type: moduleType,
      name,
    });
    setShowMenu(false);
  };

  return (
    <div className="relative">
      <motion.button
        whileHover={{ scale: 1.05 }}
        whileTap={{ scale: 0.95 }}
        onClick={() => setShowMenu(!showMenu)}
        className="w-full h-16 rounded-2xl backdrop-blur-2xl bg-white/10 border-2 border-dashed border-white/30 hover:border-white/50 hover:bg-white/15 transition-all flex items-center justify-center group"
      >
        <Plus className="size-6 text-white/70 group-hover:text-white transition-colors" />
      </motion.button>

      <AnimatePresence>
        {showMenu && (
          <motion.div
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -10 }}
            className="absolute z-50 w-full mt-2 rounded-2xl backdrop-blur-2xl bg-white/20 border border-white/30 shadow-2xl overflow-hidden"
          >
            {modules.map((module) => (
              <button
                key={module.type}
                onClick={() => handleAdd(module.type, module.name)}
                className="w-full px-4 py-3 text-left hover:bg-white/20 transition-all flex items-center gap-3 border-b border-white/10 last:border-b-0"
              >
                <module.icon className="size-5 text-white" />
                <span className="text-sm text-white font-medium">{module.name}</span>
              </button>
            ))}
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
}
