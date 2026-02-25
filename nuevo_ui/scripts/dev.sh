#!/bin/bash
#
# Development mode: Run backend and frontend in parallel
#
# Terminal 1: Backend (FastAPI with auto-reload)
# Terminal 2: Frontend (Vite dev server)
#

set -e

echo "================================"
echo "NUEVO UI - Development Mode"
echo "================================"
echo ""
echo "This script will start:"
echo "  - Backend at http://localhost:8000 (mock mode)"
echo "  - Frontend at http://localhost:5173"
echo ""
echo "Press Ctrl+C to stop both servers"
echo ""

# Function to kill background processes
cleanup() {
  echo ""
  echo "Stopping servers..."
  kill $BACKEND_PID $FRONTEND_PID 2>/dev/null || true
  exit
}

trap cleanup INT TERM

cd "$(dirname "$0")/.."

# Start backend in mock mode
echo "[1/2] Starting backend (mock mode)..."
cd backend
NUEVO_MOCK=1 python3 -m nuevo_bridge &
BACKEND_PID=$!
cd ..

# Wait for backend to start
sleep 2

# Start frontend
echo "[2/2] Starting frontend..."
cd frontend
npm run dev &
FRONTEND_PID=$!
cd ..

echo ""
echo "✓ Both servers started!"
echo "  Backend:  http://localhost:8000 (PID: $BACKEND_PID)"
echo "  Frontend: http://localhost:5173 (PID: $FRONTEND_PID)"
echo ""
echo "Open http://localhost:5173 in your browser"
echo "Press Ctrl+C to stop"
echo ""

# Wait for processes
wait
