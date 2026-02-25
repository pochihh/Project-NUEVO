"""
Entry point for running nuevo_bridge as a module
"""
import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        "nuevo_bridge.app:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info",
    )
