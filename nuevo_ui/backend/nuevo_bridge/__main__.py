"""
Entry point for running nuevo_bridge as a module.

Usage (plain Python / dev):
    python -m nuevo_bridge

Usage (ROS2 mode via colcon install):
    NUEVO_ROS2=1 nuevo_bridge
"""
import uvicorn


def main():
    uvicorn.run(
        "nuevo_bridge.app:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info",
    )


if __name__ == "__main__":
    main()
