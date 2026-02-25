import { useState } from 'react';
import { motion } from 'motion/react';
import { Camera, MapPin, Activity, X } from 'lucide-react';

type ModuleType = 'camera' | 'gps' | 'imu' | 'lidar' | 'ultrasonic';

interface ModuleCardProps {
  id: number;
  type: ModuleType;
  name: string;
  onRemove: (id: number) => void;
}

export function ModuleCard({ id, type, name, onRemove }: ModuleCardProps) {
  const getIcon = () => {
    switch (type) {
      case 'camera':
        return <Camera className="size-5 text-white" />;
      case 'gps':
        return <MapPin className="size-5 text-white" />;
      case 'imu':
      case 'lidar':
      case 'ultrasonic':
        return <Activity className="size-5 text-white" />;
      default:
        return <Activity className="size-5 text-white" />;
    }
  };

  const renderContent = () => {
    switch (type) {
      case 'camera':
        return (
          <div className="aspect-video rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 flex items-center justify-center">
            <span className="text-xs text-white/50">Camera Feed</span>
          </div>
        );
      case 'gps':
        return (
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-xs text-white/60">Latitude</span>
              <span className="text-xs font-mono text-white">37.7749° N</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-xs text-white/60">Longitude</span>
              <span className="text-xs font-mono text-white">122.4194° W</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-xs text-white/60">Altitude</span>
              <span className="text-xs font-mono text-white">16m</span>
            </div>
          </div>
        );
      case 'imu':
        return (
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-xs text-white/60">Roll</span>
              <span className="text-xs font-mono text-white">12.5°</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-xs text-white/60">Pitch</span>
              <span className="text-xs font-mono text-white">-3.2°</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-xs text-white/60">Yaw</span>
              <span className="text-xs font-mono text-white">45.0°</span>
            </div>
          </div>
        );
      default:
        return (
          <div className="text-xs text-white/50 text-center py-4">
            {name} data visualization
          </div>
        );
    }
  };

  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -20 }}
      className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl"
    >
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

      <div className="relative">
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-2">
            {getIcon()}
            <span className="text-sm font-semibold text-white">{name}</span>
          </div>
          <button
            onClick={() => onRemove(id)}
            className="p-1 rounded-lg backdrop-blur-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all"
          >
            <X className="size-4 text-white" />
          </button>
        </div>

        {renderContent()}
      </div>
    </motion.div>
  );
}
