/**
 * Modal — shared full-viewport modal wrapper.
 *
 * Uses createPortal to render directly into document.body, escaping all parent
 * stacking contexts created by backdrop-filter (e.g. the PCB card's backdrop-blur-xl).
 * z-[200] ensures it sits above the sticky header (z-50) and all other UI.
 */
import { type ReactNode } from 'react'
import { createPortal } from 'react-dom'
import { motion, AnimatePresence } from 'motion/react'
import { X } from 'lucide-react'

interface ModalProps {
  open: boolean
  onClose: () => void
  title: ReactNode
  subtitle?: ReactNode
  /** Extra content rendered in the header row, between the title and the close button */
  headerActions?: ReactNode
  /** Tailwind max-width class, e.g. "max-w-4xl". Default: "max-w-4xl" */
  maxWidth?: string
  children: ReactNode
}

export function Modal({
  open,
  onClose,
  title,
  subtitle,
  headerActions,
  maxWidth = 'max-w-4xl',
  children,
}: ModalProps) {
  return createPortal(
    <AnimatePresence>
      {open && (
        <motion.div
          key="modal-backdrop"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          transition={{ duration: 0.18 }}
          className="fixed inset-0 z-[200] flex items-center justify-center p-4 bg-black/75 backdrop-blur-md"
          onClick={onClose}
        >
          <motion.div
            key="modal-card"
            initial={{ scale: 0.93, opacity: 0, y: 12 }}
            animate={{ scale: 1, opacity: 1, y: 0 }}
            exit={{ scale: 0.93, opacity: 0, y: 12 }}
            transition={{ type: 'spring', damping: 28, stiffness: 320, mass: 0.8 }}
            onClick={(e) => e.stopPropagation()}
            className={`relative w-full ${maxWidth} max-h-[88vh] overflow-y-auto rounded-3xl p-6 bg-white/10 backdrop-blur-2xl border border-white/20 shadow-2xl`}
          >
            {/* Top shimmer line */}
            <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent rounded-t-3xl" />

            {/* Header */}
            <div className="flex items-center justify-between mb-6">
              <div>
                <h2 className="text-2xl font-bold text-white">{title}</h2>
                {subtitle && (
                  <p className="text-xs text-white/50 mt-1">{subtitle}</p>
                )}
              </div>
              <div className="flex items-center gap-3 ml-4 flex-shrink-0">
                {headerActions}
                <button
                  onClick={onClose}
                  className="p-2 rounded-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all"
                >
                  <X className="size-5 text-white" />
                </button>
              </div>
            </div>

            {children}
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>,
    document.body,
  )
}
