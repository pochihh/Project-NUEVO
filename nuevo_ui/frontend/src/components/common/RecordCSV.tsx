/**
 * RecordCSV Component
 *
 * States:
 *   idle      → [Record CSV] button
 *   recording → [● Stop Recording] button; plot shows red bar overlay
 *   preparing → spinner (brief async while slicing data)
 *   ready     → [filename input] [Download] [×] row
 */
import { useState } from 'react'
import { Download, X } from 'lucide-react'
import { downloadCSV } from '../../lib/csvExport'
import './RecordCSV.css'

interface RecordCSVProps {
  headers: string[]
  getData: () => number[][]
  filename: string
  /** Called when recording state changes so parent can update the plot overlay */
  onRecordingChange: (isRecording: boolean) => void
}

export function RecordCSV({ headers, getData, filename, onRecordingChange }: RecordCSVProps) {
  const [isRecording, setIsRecording] = useState(false)
  const [isPreparing, setIsPreparing] = useState(false)
  const [pendingData, setPendingData] = useState<number[][] | null>(null)
  const [editedFilename, setEditedFilename] = useState('')

  const handleStartRecording = () => {
    setPendingData(null)
    setIsRecording(true)
    onRecordingChange(true)
  }

  const handleStopRecording = () => {
    // Snapshot data BEFORE clearing recording state so getData() slices from recordingStartTs
    const data = getData()
    onRecordingChange(false)
    setIsRecording(false)
    setIsPreparing(true)

    // Brief async to let the spinner render before potentially heavy data processing
    setTimeout(() => {
      const now = new Date()
      const mm = String(now.getMonth() + 1).padStart(2, '0')
      const dd = String(now.getDate()).padStart(2, '0')
      const hh = String(now.getHours()).padStart(2, '0')
      const min = String(now.getMinutes()).padStart(2, '0')
      setEditedFilename(`${filename}_${mm}${dd}_${hh}-${min}.csv`)
      setPendingData(data)
      setIsPreparing(false)
    }, 50)
  }

  const handleDownload = () => {
    if (!pendingData) return
    downloadCSV(headers, pendingData, editedFilename)
    setPendingData(null)
  }

  const handleDiscard = () => {
    setPendingData(null)
  }

  // ── Preparing spinner ──────────────────────────────────────────────────────
  if (isPreparing) {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-xl bg-white/10 border border-white/20 text-xs text-white/50">
        <div className="size-3 rounded-full border-2 border-white/30 border-t-white/80 animate-spin flex-shrink-0" />
        Preparing…
      </div>
    )
  }

  // ── Download row ───────────────────────────────────────────────────────────
  if (pendingData) {
    return (
      <div className="flex items-center gap-2">
        <input
          type="text"
          value={editedFilename}
          onChange={(e) => setEditedFilename(e.target.value)}
          className="flex-1 max-w-80 text-xs bg-white/10 border border-white/20 rounded-xl px-3 py-1.5 text-white/80 focus:outline-none focus:border-cyan-400/50 font-mono"
          spellCheck={false}
        />
        <button
          onClick={handleDownload}
          className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/40 transition-all text-xs font-semibold text-white flex-shrink-0"
        >
          <Download className="size-3" />
          Download
        </button>
        <button
          onClick={handleDiscard}
          className="p-1.5 rounded-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all flex-shrink-0"
          title="Discard"
        >
          <X className="size-3.5 text-white/50" />
        </button>
      </div>
    )
  }

  // ── Record / Stop button ───────────────────────────────────────────────────
  return (
    <button
      className={`record-csv-btn ${isRecording ? 'recording' : ''}`}
      onClick={isRecording ? handleStopRecording : handleStartRecording}
    >
      {isRecording ? (
        <>
          <span className="record-dot" />
          Stop Recording
        </>
      ) : (
        'Record CSV'
      )}
    </button>
  )
}
