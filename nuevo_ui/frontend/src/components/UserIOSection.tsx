import { useState } from "react";
import { Plus, X } from "lucide-react";
import { Switch } from "./ui/switch";
import { Slider } from "./ui/slider";
import { createPortal } from "react-dom";
import { useRobotStore } from "../store/robotStore";
import { wsSend } from "../lib/wsSend";

// Fixed button layout — order defines visual position in the JSX below
const BUTTON_LAYOUT = [7, 6, 9, 8, 10, 1, 2, 3, 4, 5];

// 8 limit switches displayed in 2×4 grid (IDs 1-8)
const LIMIT_SW_LAYOUT = [1, 5, 2, 6, 3, 7, 4, 8];

interface LED {
  name: string;
  color: string;
  index: number; // 0-based command index
}

const USER_LEDS: LED[] = [
  { name: "Red",    color: "#ef4444", index: 0 },
  { name: "Green",  color: "#10b981", index: 1 },
  { name: "Blue",   color: "#3b82f6", index: 2 },
  { name: "Orange", color: "#f97316", index: 3 },
  { name: "Purple", color: "#a855f7", index: 4 },
];

interface WS2812B {
  id: number;
  r: number;
  g: number;
  b: number;
  brightness: number; // 0-100 %
  hue: number;
}

export function UserIOSection() {
  const io     = useRobotStore((s) => s.io);
  const system = useRobotStore((s) => s.system);

  // LED state — local (we send commands; no per-LED state in incoming data)
  const [ledEnabled, setLedEnabled] = useState<boolean[]>(Array(5).fill(false));

  // NeoPixel strips
  const [strips, setStrips] = useState<WS2812B[]>([
    { id: 1, r: 255, g: 0, b: 0, brightness: 50, hue: 0 },
  ]);
  const [showPicker, setShowPicker] = useState<number | null>(null);
  const [pickerPos, setPickerPos] = useState({ x: 0, y: 0 });

  // Button state from bitmask
  const buttonMask      = io?.buttonMask ?? 0;
  const limitSwitchMask = system?.limitSwitchMask ?? 0;

  const isButtonPressed     = (id: number) => ((buttonMask >> (id - 1)) & 1) === 1;
  const isLimitPressed      = (id: number) => ((limitSwitchMask >> (id - 1)) & 1) === 1;

  const toggleLed = (index: number) => {
    const next = !ledEnabled[index];
    setLedEnabled((prev) => prev.map((v, i) => (i === index ? next : v)));
    wsSend('set_led', { index, state: next });
  };

  const addStrip = () => {
    setStrips((prev) => [...prev, { id: prev.length + 1, r: 0, g: 0, b: 0, brightness: 0, hue: 0 }]);
  };

  const hslToRgb = (hue: number, brightness: number): [number, number, number] => {
    const l = brightness / 100;
    const s = 1;
    const c = (1 - Math.abs(2 * l - 1)) * s;
    const x = c * (1 - Math.abs(((hue / 60) % 2) - 1));
    const m = l - c / 2;
    let r = 0, g = 0, b = 0;
    if (hue < 60)       { r = c; g = x; b = 0; }
    else if (hue < 120) { r = x; g = c; b = 0; }
    else if (hue < 180) { r = 0; g = c; b = x; }
    else if (hue < 240) { r = 0; g = x; b = c; }
    else if (hue < 300) { r = x; g = 0; b = c; }
    else                { r = c; g = 0; b = x; }
    return [Math.round((r + m) * 255), Math.round((g + m) * 255), Math.round((b + m) * 255)];
  };

  const rgbToHex = (r: number, g: number, b: number) =>
    '#' + [r, g, b].map((x) => x.toString(16).padStart(2, '0')).join('');

  const updateStrip = (id: number, updates: Partial<WS2812B>) => {
    setStrips((prev) =>
      prev.map((s) => {
        if (s.id !== id) return s;
        const next = { ...s, ...updates };
        if (updates.hue !== undefined || updates.brightness !== undefined) {
          const [r, g, b] = hslToRgb(next.hue, next.brightness);
          next.r = r; next.g = g; next.b = b;
          // Send command for strip index 0-based (strip.id - 1)
          wsSend('set_neopixel', {
            index: id - 1,
            red: r, green: g, blue: b,
          });
        }
        return next;
      })
    );
  };

  const handleHueClick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (showPicker === null) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - rect.left - rect.width / 2;
    const y = e.clientY - rect.top - rect.height / 2;
    let angle = (Math.atan2(y, x) * 180) / Math.PI;
    angle = (angle + 360) % 360;
    updateStrip(showPicker, { hue: angle });
  };

  const handleLEDClick = (e: React.MouseEvent<HTMLButtonElement>, stripId: number) => {
    const rect = e.currentTarget.getBoundingClientRect();
    setPickerPos({ x: rect.left, y: rect.bottom + 8 });
    setShowPicker(showPicker === stripId ? null : stripId);
  };

  const btnClass = (pressed: boolean) =>
    `flex items-center justify-center text-white text-xs size-6 rounded-full border-2 transition-all ${
      pressed
        ? "border-white/60 bg-white/50 shadow-[0_0_10px_rgba(255,255,255,0.25)]"
        : "bg-white/10 border-white/30"
    }`;

  const lsClass = (pressed: boolean) =>
    `flex items-center justify-center text-white text-xs h-6 rounded-lg border transition-all ${
      pressed
        ? "border-white/60 bg-white/50 shadow-[0_0_10px_rgba(255,255,255,0.25)]"
        : "bg-white/10 border-white/20"
    }`;

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

      <div className="relative space-y-4">
        <div className="grid grid-cols-2">

          {/* Limit Switches */}
          <div>
            <h4 className="text-sm font-semibold text-white mb-2">Limit switches</h4>
            <div className="grid grid-cols-2 gap-1.5">
              {LIMIT_SW_LAYOUT.map((id) => (
                <div key={id} className={lsClass(isLimitPressed(id))}>{id}</div>
              ))}
            </div>
          </div>

          {/* User LEDs */}
          <div className="ml-10">
            <h4 className="text-sm font-semibold text-white mb-2">LEDs</h4>
            <div className="space-y-2">
              {USER_LEDS.map((led, i) => (
                <div key={led.name} className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <div
                      className="size-3 rounded-full"
                      style={{
                        backgroundColor: ledEnabled[i] ? led.color : "rgba(255,255,255,0.2)",
                        boxShadow: ledEnabled[i] ? `0 0 8px ${led.color}` : "none",
                      }}
                    />
                    <span className="text-xs text-white/70">{led.name}</span>
                  </div>
                  <Switch
                    checked={ledEnabled[i]}
                    onCheckedChange={() => toggleLed(i)}
                  />
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Buttons */}
        <div>
          <h4 className="text-sm font-semibold text-white mb-2">Buttons</h4>
          <div className="flex flex-col items-center gap-2">
            {/* Row 1 — B7 */}
            <div className={btnClass(isButtonPressed(BUTTON_LAYOUT[0]))}>{BUTTON_LAYOUT[0]}</div>

            {/* Row 2 — B6, B9, B8 */}
            <div className="flex gap-2">
              {[1, 2, 3].map((i) => (
                <div key={i} className={btnClass(isButtonPressed(BUTTON_LAYOUT[i]))}>{BUTTON_LAYOUT[i]}</div>
              ))}
            </div>

            {/* Row 3 — B10 */}
            <div className={btnClass(isButtonPressed(BUTTON_LAYOUT[4]))}>{BUTTON_LAYOUT[4]}</div>

            {/* Row 4 — B1–B5 */}
            <div className="flex gap-2">
              {[5, 6, 7, 8, 9].map((i) => (
                <div key={i} className={btnClass(isButtonPressed(BUTTON_LAYOUT[i]))}>{BUTTON_LAYOUT[i]}</div>
              ))}
            </div>
          </div>
        </div>

        {/* WS2812B RGB LEDs */}
        <div className="relative">
          <div className="flex items-center gap-2 flex-wrap">
            <h4 className="text-sm font-semibold text-white">RGB LED</h4>

            {strips.map((strip) => (
              <button
                key={strip.id}
                onClick={(e) => handleLEDClick(e, strip.id)}
                className="size-6 rounded-lg border-2 border-white/30 transition-all hover:scale-110"
                style={{
                  backgroundColor: rgbToHex(strip.r, strip.g, strip.b),
                  boxShadow: `0 0 8px ${rgbToHex(strip.r, strip.g, strip.b)}`,
                }}
              />
            ))}

            <button
              onClick={addStrip}
              className="size-6 rounded-lg backdrop-blur-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all flex items-center justify-center"
            >
              <Plus className="size-3 text-white" />
            </button>
          </div>
        </div>
      </div>

      {/* Color picker popout */}
      {showPicker !== null &&
        strips.find((s) => s.id === showPicker) &&
        createPortal(
          <div className="fixed z-50" style={{ left: pickerPos.x, top: pickerPos.y }}>
            <div
              className="ml-3 w-0 h-0"
              style={{
                borderLeft: "6px solid transparent",
                borderRight: "6px solid transparent",
                borderBottom: "6px solid rgba(255, 255, 255, 0.2)",
              }}
            />
            <div className="rounded-2xl backdrop-blur-2xl bg-white/10 border border-white/20 shadow-2xl p-4">
              <div className="flex justify-end mb-2">
                <button
                  onClick={() => setShowPicker(null)}
                  className="p-1 rounded-lg backdrop-blur-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all"
                >
                  <X className="size-3 text-white" />
                </button>
              </div>

              {/* Hue ring */}
              <div className="flex justify-center mb-4">
                <div
                  onClick={handleHueClick}
                  className="relative w-32 h-32 rounded-full cursor-pointer"
                  style={{
                    background: `conic-gradient(
                      hsl(0,100%,50%), hsl(30,100%,50%), hsl(60,100%,50%),
                      hsl(90,100%,50%), hsl(120,100%,50%), hsl(150,100%,50%),
                      hsl(180,100%,50%), hsl(210,100%,50%), hsl(240,100%,50%),
                      hsl(270,100%,50%), hsl(300,100%,50%), hsl(330,100%,50%),
                      hsl(360,100%,50%))`,
                  }}
                >
                  <div className="absolute inset-4 rounded-full backdrop-blur-xl bg-white/10 border border-white/20" />
                  <div
                    className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 size-8 rounded-full border-2 border-white shadow-lg"
                    style={{
                      backgroundColor: rgbToHex(
                        strips.find((s) => s.id === showPicker)!.r,
                        strips.find((s) => s.id === showPicker)!.g,
                        strips.find((s) => s.id === showPicker)!.b,
                      ),
                    }}
                  />
                </div>
              </div>

              {/* Brightness slider */}
              <div className="space-y-2 w-48">
                <div className="flex items-center justify-between">
                  <span className="text-xs text-white/70">Brightness</span>
                  <span className="text-xs font-mono text-white">
                    {strips.find((s) => s.id === showPicker)?.brightness ?? 0}%
                  </span>
                </div>
                <Slider
                  value={[strips.find((s) => s.id === showPicker)?.brightness ?? 0]}
                  onValueChange={(val) => updateStrip(showPicker!, { brightness: val[0] })}
                  min={0}
                  max={100}
                  step={1}
                />
              </div>
            </div>
          </div>,
          document.body,
        )}
    </div>
  );
}
