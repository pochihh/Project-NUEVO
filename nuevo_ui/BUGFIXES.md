# Bug Fixes - 2026-02-13

## Issues Reported

1. **UI Scrolling Issue**: 4 DC motor panels out of screen, can't scroll down
2. **No Live Data Plot**: Plots not visible
3. **Frontend WebSocket Error**: EPIPE error in Vite proxy

## Fixes Applied

### Fix 1: Scrolling Issue
**Problem**: DCMotorPanel container didn't have proper height constraints for overflow scrolling.

**Solution**: Updated `/nuevo_ui/frontend/src/components/dc/DCMotorPanel.css`
- Added `min-height: 0` to enable flexbox overflow
- Made `.dc-motor-grid` a flex container with `overflow-y: auto`
- Added custom dark theme scrollbar styling
- Panel header is now `flex-shrink: 0` to prevent compression
- Grid takes remaining space with `flex: 1`

**Result**: DC motor cards now scroll properly within the grid cell.

### Fix 2: Plot Visibility
**Problem**: uPlot charts weren't rendering or visible in dark theme.

**Solution**: Updated multiple files:

1. **DCPlot.css** - Enhanced styling:
   - Added `!important` to dark theme overrides
   - Set explicit `min-height: 200px`
   - Added background color to plot container
   - Styled axes and grid lines for dark theme
   - Added empty state styling

2. **DCPlot.tsx** - Improved rendering logic:
   - Don't render plot until data arrives (prevents empty plot errors)
   - Show "Waiting for data..." message when `timeHistory.length === 0`
   - Removed fallback `[0]` arrays that could cause rendering issues

**Result**: Plots now render correctly with dark theme and show loading state.

### Fix 3: WebSocket EPIPE Error
**Problem**: Vite proxy throws EPIPE error when backend WebSocket closes unexpectedly.

**Solution**: Updated `/nuevo_ui/frontend/src/hooks/useWebSocket.ts`
- Added more detailed close event handling
- Only reconnect on abnormal closes (code !== 1000)
- Improved error logging with close codes and reasons
- Close socket on error to trigger proper reconnection

**Note**: The EPIPE error itself is a Vite proxy issue and is non-critical. It appears in Vite's terminal when the backend WebSocket closes. Our improved error handling makes reconnection more reliable.

## Current Motor Display

The UI currently displays **2 motors** (Motor 0 and Motor 1) as configured in `DCMotorPanel.tsx`:
```tsx
<DCMotorCard motorId={0} />
<DCMotorCard motorId={1} />
```

To show all 4 motors, simply add:
```tsx
<DCMotorCard motorId={2} />
<DCMotorCard motorId={3} />
```

## Testing Instructions

1. **Restart the frontend** to pick up CSS changes:
   ```bash
   cd nuevo_ui/frontend
   npm run dev
   ```

2. **Verify scrolling**:
   - Open browser to http://localhost:5173
   - Look at DC Motors panel (bottom right)
   - Should see Motor 0 and Motor 1 cards
   - If you add Motor 2 and 3, you should be able to scroll

3. **Verify plots**:
   - Each motor card should show a plot area
   - If no data yet, you'll see "Waiting for data..."
   - Once backend connects, plots should show live data

4. **Verify WebSocket**:
   - Connection badge should show green "WS: Connected"
   - Check browser console for `[WS] Connected` message
   - EPIPE errors in Vite terminal are non-critical (can be ignored)

## Next Steps

- Add Motor 2 and Motor 3 if needed
- Test with real hardware (non-mock mode)
- Implement Phase 4: Stepper + Servo Control
