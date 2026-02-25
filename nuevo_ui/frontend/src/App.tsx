import { useState } from 'react';
import { AnimatePresence } from 'motion/react';
import { Wifi, WifiOff, AlertTriangle } from 'lucide-react';
import screwLogo from './assets/screw_logo.svg';
import { PowerSection } from './components/PowerSection';
import { StepperSection } from './components/StepperSection';
import { ServoSection } from './components/ServoSection';
import { RpiSystemSection } from './components/RpiSystemSection';
import { ArduinoSystemSection } from './components/ArduinoSystemSection';
import { UserIOSection } from './components/UserIOSection';
import { DCMotorSection } from './components/DCMotorSection';
import { AddModuleButton } from './components/AddModuleButton';
import { ModuleCard } from './components/ModuleCard';
import { useWebSocket } from './hooks/useWebSocket';
import { useRobotStore } from './store/robotStore';

interface Module {
  id: number;
  type: string;
  name: string;
}

export default function App() {
  const { send } = useWebSocket();
  const connected       = useRobotStore((s) => s.connected);
  const serialConnected = useRobotStore((s) => s.serialConnected);

  const handleEstop = () => send('sys_cmd', { command: 4 });

  const [qwiicModules, setQwiicModules] = useState<Module[]>([]);
  const [sensorModules, setSensorModules] = useState<Module[]>([]);

  const addQwiicModule = (module: Module) => {
    setQwiicModules((prev) => [...prev, module]);
  };

  const addSensorModule = (module: Module) => {
    setSensorModules((prev) => [...prev, module]);
  };

  const removeQwiicModule = (id: number) => {
    setQwiicModules((prev) => prev.filter((m) => m.id !== id));
  };

  const removeSensorModule = (id: number) => {
    setSensorModules((prev) => prev.filter((m) => m.id !== id));
  };

  return (
    <div className="min-h-screen w-full relative overflow-hidden">
      {/* Background with gradient */}
      {/* <div className="absolute inset-0 bg-gradient-to-br from-indigo-900 via-purple-900 to-pink-900"> */}
      <div className="absolute inset-0" style={{
        background: 'linear-gradient(45deg, #292E49, #536976)'
      }}>
        
        {/* Animated background blobs */}
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-cyan-400/20 rounded-full mix-blend-multiply filter blur-3xl animate-blob"></div>
          <div className="absolute top-1/3 right-1/4 w-96 h-96 bg-purple-400/20 rounded-full mix-blend-multiply filter blur-3xl animate-blob animation-delay-2000"></div>
          <div className="absolute bottom-1/4 right-1/3 w-96 h-96 bg-pink-400/20 rounded-full mix-blend-multiply filter blur-3xl animate-blob animation-delayˋ000"></div>
        </div>
      </div>

      {/* Glass container */}
      <div className="relative z-10 min-h-screen">
        {/* Top Navigation Bar */}
        <div className="sticky top-0 z-50 backdrop-blur-2xl bg-white/10 border-b border-white/20 shadow-lg">
          <div className="max-w-[1800px] mx-auto px-6 py-4 flex items-center justify-between">
            {/* Left side */}
            <div className="flex items-center gap-6">
              {/* <button className="p-2 rounded-xl backdrop-blur-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all">
                <Menu className="size-5 text-white" />
              </button> */}
              
              <div className="flex items-center gap-3">
                <div className="size-10 rounded-2xl backdrop-blur-xl bg-gradient-to-br from-white/20 to-white/10 border border-white/30 shadow-lg flex items-center justify-center">
                  <img src={screwLogo} className="size-6 invert" alt="logo" />
                </div>
                <div>
                  <h1 className="text-xl font-bold text-white">NUEVO UI</h1>
                  <p className="text-xs text-white/70">For the NUEVO Board</p>
                </div>
              </div>

              {/* Search bar */}
              {/* <div className="ml-8 hidden lg:flex items-center gap-3 px-4 py-2.5 rounded-2xl backdrop-blur-xl bg-white/10 border border-white/20 min-w-[300px]">
                <Search className="size-4 text-white/50" />
                <input
                  type="text"
                  placeholder="Search..."
                  className="bg-transparent border-none outline-none text-white placeholder:text-white/50 flex-1 text-sm"
                />
              </div> */}
            </div>

            {/* Right side */}
            <div className="flex items-center gap-3">
              {/* Connection badge */}
              <div className="flex items-center gap-2 px-3 py-1.5 rounded-xl backdrop-blur-xl bg-white/10 border border-white/20">
                {connected ? (
                  <Wifi className="size-4 text-emerald-400" />
                ) : (
                  <WifiOff className="size-4 text-rose-400" />
                )}
                <span className="text-xs text-white/80">
                  {!connected ? 'Disconnected' : !serialConnected ? 'UI only' : 'Serial OK'}
                </span>
              </div>

              {/* E-STOP — always visible when serial connected */}
              {serialConnected && (
                <button
                  onClick={handleEstop}
                  className="flex items-center gap-2 px-4 py-2 rounded-xl backdrop-blur-xl bg-red-500/50 border-2 border-red-400/80 hover:bg-red-500/70 active:scale-95 transition-all text-sm font-black text-white tracking-wide shadow-lg shadow-red-500/30"
                >
                  <AlertTriangle className="size-5" />
                  E-STOP
                </button>
              )}
            </div>
          </div>
        </div>

        {/* Main Content */}
        <div className="max-w-[1800px] mx-auto p-6">
          {/* <div className="mb-6">
            <h2 className="text-3xl font-bold text-white mb-2">Dashboard Overview</h2>
            <p className="text-white/70">Real-time robotic system control and monitoring</p>
          </div> */}

          {/* PCB Board Container - Centered with padding */}
          <div className="flex justify-center mb-8">
            <div className="w-full max-w-[1600px] rounded-3xl p-8 backdrop-blur-xl bg-white/5 border-2 border-white/20 shadow-2xl">
              <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
              
              {/* PCB Board Layout */}
              <div className="space-y-8">
                {/* Top Row: Power, Steppers, Servos */}
                <div className="grid grid-cols-1 md:grid-cols-8 gap-4">
                  <div className="md:col-span-2">
                    <PowerSection />
                  </div>
                  <div className="md:col-span-1">
                    <StepperSection stepperId={1} />
                  </div>
                  <div className="md:col-span-1">
                    <StepperSection stepperId={2} />
                  </div>
                  <div className="md:col-span-1">
                    <StepperSection stepperId={3} />
                  </div>
                  <div className="md:col-span-1">
                    <StepperSection stepperId={4} />
                  </div>
                  <div className="md:col-span-2">
                    <ServoSection />
                  </div>
                </div>

                {/* Middle Row: Rpi, Arduino, User IO, DC Motors */}
                <div className="grid grid-cols-1 md:grid-cols-8 gap-4">
                  {/* Rpi System - 1 column */}
                  <div className="md:col-span-2">
                    <RpiSystemSection />
                  </div>

                  {/* Arduino System - 1 column */}
                  <div className="md:col-span-2">
                    <ArduinoSystemSection />
                  </div>

                  {/* User IO - 2 columns */}
                  <div className="md:col-span-2">
                    <UserIOSection />
                  </div>

                  {/* DC Motors - 2 columns */}
                  <div className="md:col-span-2 space-y-4">
                    <DCMotorSection motorId={4} />
                    <DCMotorSection motorId={3} />
                    <DCMotorSection motorId={2} />
                    <DCMotorSection motorId={1} />
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Expandable Modules - Outside PCB Layout */}
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 max-w-[1600px] mx-auto">
            {/* Qwiic Modules Column */}
            <div className="space-y-4">
              <h3 className="text-lg font-semibold text-white mb-3">Qwiic Modules</h3>
              <AnimatePresence>
                {qwiicModules.map((module) => (
                  <ModuleCard
                    key={module.id}
                    {...module}
                    onRemove={removeQwiicModule}
                  />
                ))}
              </AnimatePresence>
              <AddModuleButton type="qwiic" onAdd={addQwiicModule} />
            </div>

            {/* Sensor Modules Column */}
            <div className="space-y-4">
              <h3 className="text-lg font-semibold text-white mb-3">Sensor Modules</h3>
              <AnimatePresence>
                {sensorModules.map((module) => (
                  <ModuleCard
                    key={module.id}
                    {...module}
                    onRemove={removeSensorModule}
                  />
                ))}
              </AnimatePresence>
              <AddModuleButton type="sensor" onAdd={addSensorModule} />
            </div>

            {/* Placeholder for third column */}
            <div className="hidden xl:block" />
          </div>
        </div>
      </div>

      <style>{`
        @keyframes blob {
          0% { transform: translate(0px, 0px) scale(1); }
          33% { transform: translate(30px, -50px) scale(1.1); }
          66% { transform: translate(-20px, 20px) scale(0.9); }
          100% { transform: translate(0px, 0px) scale(1); }
        }
        .animate-blob {
          animation: blob 7s infinite;
        }
        .animation-delay-2000 {
          animation-delay: 2s;
        }
        .animation-delay-4000 {
          animation-delay: 4s;
        }
      `}</style>
    </div>
  );
}
