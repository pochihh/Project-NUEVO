/**
 * DCPlot Component
 *
 * Real-time uPlot chart showing position, velocity, current, and PWM duty cycle.
 * X-axis = "seconds ago": newest data at left (0), oldest at right.
 *
 * The animation loop uses refs for all data so it never restarts — a single
 * persistent rAF runs at 60fps for smooth overlay animation even between data
 * updates.
 *
 * Default window: 20 s.
 * While recording: window stretches smoothly so the recording-start bar stays
 * pinned at the right edge (xMax = elapsed recording time).
 *
 * Recording overlay:
 *   - Red dashed vertical bar at the recording-start position (right edge)
 *   - Semi-transparent red fill covering the entire recorded region
 */
import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import './DCPlot.css'

interface DCPlotProps {
  timeHistory: number[]       // bridge receive time in ms
  positionHistory: number[]
  velocityHistory: number[]
  currentHistory: number[]
  pwmHistory: number[]
  recordingStartTs: number | null  // bridge time ms when recording started; null = not recording
}

export function DCPlot({
  timeHistory,
  positionHistory,
  velocityHistory,
  currentHistory,
  pwmHistory,
  recordingStartTs,
}: DCPlotProps) {
  const chartRef = useRef<HTMLDivElement>(null)
  const plotRef  = useRef<uPlot | null>(null)
  const animRef  = useRef<number | null>(null)
  const seriesVisRef = useRef<boolean[]>([true, true, true, true])

  // Data refs — updated every render so the single rAF loop always sees latest values
  const timeRef       = useRef(timeHistory)
  const posRef        = useRef(positionHistory)
  const velRef        = useRef(velocityHistory)
  const curRef        = useRef(currentHistory)
  const pwmRef        = useRef(pwmHistory)
  const recStartRef   = useRef(recordingStartTs)
  const xMaxRef       = useRef(20)

  useEffect(() => { timeRef.current     = timeHistory      }, [timeHistory])
  useEffect(() => { posRef.current      = positionHistory  }, [positionHistory])
  useEffect(() => { velRef.current      = velocityHistory  }, [velocityHistory])
  useEffect(() => { curRef.current      = currentHistory   }, [currentHistory])
  useEffect(() => { pwmRef.current      = pwmHistory       }, [pwmHistory])
  useEffect(() => { recStartRef.current = recordingStartTs }, [recordingStartTs])

  // Create plot once on mount
  useEffect(() => {
    if (!chartRef.current) return

    const opts: uPlot.Options = {
      width:  chartRef.current.clientWidth,
      height: 200,
      series: [
        { label: 'Seconds Ago' },
        { label: 'Position (ticks)', stroke: '#4ade80', width: 2, scale: 'motion',  show: true },
        { label: 'Velocity (t/s)',   stroke: '#3b82f6', width: 2, scale: 'motion',  show: true },
        { label: 'Current (mA)',     stroke: '#f59e0b', width: 2, scale: 'current', show: true },
        { label: 'Duty Cycle (%)',   stroke: '#ec4899', width: 2, scale: 'duty',    show: true },
      ],
      scales: {
        x: {
          time: false,
          // Range reads the ref — updated each frame for smooth stretching
          range: (_u: uPlot, _dmin: number, _dmax: number) => [0, xMaxRef.current],
        },
        motion:  { auto: true },
        current: {
          auto: true,
          range: (_u: uPlot, _dmin: number, dmax: number) => [0, Math.max(100, dmax * 1.1)],
        },
        duty: { auto: false, range: [0, 100] },
      },
      axes: [
        { label: 'Seconds Ago',          stroke: '#9ca3af', grid: { stroke: '#333', width: 1 } },
        { label: 'Position / Velocity',  stroke: '#4ade80', grid: { stroke: '#333', width: 1 }, scale: 'motion',  side: 3 },
        { label: 'Current (mA)',         stroke: '#f59e0b', grid: { show: false },              scale: 'current', side: 1 },
        { label: 'Duty %',               stroke: '#ec4899', grid: { show: false },              scale: 'duty',    side: 1 },
      ],
      legend: { show: true, live: false },
      cursor: { drag: { x: false, y: false } },
      hooks: {
        draw: [
          (u: uPlot) => {
            const startTs = recStartRef.current
            if (!startTs) return

            const th = timeRef.current
            if (th.length === 0) return

            const elapsed = (th[th.length - 1] - startTs) / 1000
            if (elapsed <= 0) return

            // canvas-pixel coordinates (true = canvas px, matches u.bbox)
            const x0   = Math.round(u.valToPos(0,       'x', true))  // left edge (newest)
            const barX = Math.round(u.valToPos(elapsed, 'x', true))  // recording-start bar
            const top    = u.bbox.top
            const height = u.bbox.height

            const ctx = u.ctx
            ctx.save()
            ctx.fillStyle = 'rgba(239, 68, 68, 0.08)'
            ctx.fillRect(x0, top, barX - x0, height)
            ctx.strokeStyle = 'rgba(239, 68, 68, 0.75)'
            ctx.lineWidth = 1.5
            ctx.setLineDash([5, 4])
            ctx.beginPath()
            ctx.moveTo(barX, top)
            ctx.lineTo(barX, top + height)
            ctx.stroke()
            ctx.restore()
          },
        ],
      },
    }

    const emptyData: uPlot.AlignedData = [
      new Float64Array(0),
      new Float64Array(0),
      new Float64Array(0),
      new Float64Array(0),
      new Float64Array(0),
    ]

    const plot = new uPlot(opts, emptyData, chartRef.current)
    plotRef.current = plot

    // Legend series toggle
    const legend = plot.root.querySelector('.u-legend')
    if (legend) {
      legend.addEventListener('click', (e) => {
        const p = plotRef.current
        if (!p) return
        const target = (e.target as HTMLElement).closest('.u-series') as HTMLElement | null
        if (!target) return
        const items = legend.querySelectorAll('.u-series')
        const idx = Array.from(items).indexOf(target)
        if (idx < 0) return
        seriesVisRef.current[idx] = !seriesVisRef.current[idx]
        p.setSeries(idx + 1, { show: seriesVisRef.current[idx] })
      })
    }

    return () => {
      plot.destroy()
      plotRef.current = null
    }
  }, [])

  // Single persistent animation loop — never restarts, reads all data from refs.
  // This avoids the gap/jitter that occurs when the effect restarts on every data update.
  useEffect(() => {
    const tick = () => {
      const plot = plotRef.current
      const th = timeRef.current

      if (plot && th.length > 0) {
        const n      = th.length
        const newest = th[n - 1]

        // Update dynamic x-axis range smoothly:
        //   - not recording: fixed 20 s window
        //   - recording: range = exact elapsed time, so the bar stays at the right edge
        const startTs = recStartRef.current
        xMaxRef.current = startTs !== null
          ? Math.max(20, (newest - startTs) / 1000)
          : 20

        // Build plot data arrays (reversed so x is ascending)
        const xArr    = new Float64Array(n)
        const posArr  = new Float64Array(n)
        const velArr  = new Float64Array(n)
        const curArr  = new Float64Array(n)
        const dutyArr = new Float64Array(n)

        const pos = posRef.current
        const vel = velRef.current
        const cur = curRef.current
        const pwm = pwmRef.current

        for (let i = 0; i < n; i++) {
          const j    = n - 1 - i
          xArr[i]    = (newest - th[j]) / 1000
          posArr[i]  = pos[j]
          velArr[i]  = vel[j]
          curArr[i]  = cur[j]
          dutyArr[i] = (Math.abs(pwm[j]) / 255) * 100
        }

        plot.setData([xArr, posArr, velArr, curArr, dutyArr])
      }

      animRef.current = requestAnimationFrame(tick)
    }

    animRef.current = requestAnimationFrame(tick)
    return () => {
      if (animRef.current) cancelAnimationFrame(animRef.current)
    }
  }, []) // no deps — single loop for the component lifetime

  // Resize handler
  useEffect(() => {
    const onResize = () => {
      if (plotRef.current && chartRef.current) {
        plotRef.current.setSize({ width: chartRef.current.clientWidth, height: 200 })
      }
    }
    window.addEventListener('resize', onResize)
    return () => window.removeEventListener('resize', onResize)
  }, [])

  return (
    <div ref={chartRef} className="dc-plot">
      {timeHistory.length === 0 && (
        <div className="dc-plot-empty">Waiting for data...</div>
      )}
    </div>
  )
}
