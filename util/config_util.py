from phoenix6 import StatusCode


def apply_config(motor, cfg, name: str = "", retries: int = 5) -> StatusCode:
    """Apply config with retry, following CTRE's official pattern."""
    status = StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(retries):
        status = motor.configurator.apply(cfg)
        if status.is_ok():
            break
    if not status.is_ok():
        print(f"[Config] Failed to apply to {name}: {status.name}")
    return status
