import wpilib

from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX

from util.nt_util import NTTable

# Feedforward + feedback gains tracked per slot
_GAINS = ("k_p", "k_i", "k_d", "k_s", "k_v", "k_a", "k_g")


def _slot_snapshot(slot_cfg) -> dict:
    return {g: getattr(slot_cfg, g) for g in _GAINS}


def _read_slot_nt(nt: NTTable, slot: str) -> dict:
    return {g: nt.get(f"{slot}/{g}") for g in _GAINS}


def _register_slot_nt(nt: NTTable, slot: str, slot_cfg) -> None:
    for g in _GAINS:
        nt.float(f"{slot}/{g}", getattr(slot_cfg, g))


def _apply_to_slot(slot_cfg, values: dict) -> None:
    for g, v in values.items():
        setattr(slot_cfg, g, v)


class EditablePID:
    def __init__(
        self,
        name: str,
        talon_motor: TalonFX,
        cfg: TalonFXConfiguration,
        use_slot0: bool = True,
        use_slot1: bool = False,
        use_slot2: bool = False,
    ):
        self.name = name
        self.talon_motor = talon_motor
        self.cfg = cfg

        self.use_slot0 = use_slot0
        self.use_slot1 = use_slot1
        self.use_slot2 = use_slot2

        self.nt = NTTable(self.name).get_subtable("EditablePID")

        # Track last applied values to avoid unnecessary config updates
        self.last_applied_values: dict[str, dict] = {}
        # Debounce: stay below Phoenix 6's 1-second frequent-call threshold
        self._last_apply_time = -10.0

        if self.use_slot0:
            _register_slot_nt(self.nt, "slot0", self.cfg.slot0)
            self.last_applied_values["slot0"] = _slot_snapshot(self.cfg.slot0)
        if self.use_slot1:
            _register_slot_nt(self.nt, "slot1", self.cfg.slot1)
            self.last_applied_values["slot1"] = _slot_snapshot(self.cfg.slot1)
        if self.use_slot2:
            _register_slot_nt(self.nt, "slot2", self.cfg.slot2)
            self.last_applied_values["slot2"] = _slot_snapshot(self.cfg.slot2)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _slot_changed(self, slot: str, new_values: dict, tolerance: float = 1e-6) -> bool:
        return any(
            abs(new_values[g] - self.last_applied_values[slot][g]) > tolerance
            for g in _GAINS
        )

    def _check_slot(self, slot: str, slot_cfg) -> bool:
        """Read NT, detect changes, write back to cfg object. Returns True if changed."""
        new = _read_slot_nt(self.nt, slot)
        if self._slot_changed(slot, new):
            _apply_to_slot(slot_cfg, new)
            self.last_applied_values[slot] = dict(new)
            gains_str = "  ".join(f"{g}={v}" for g, v in new.items() if v != 0.0)
            print(f"[{self.name}] {slot.capitalize()} gains changed: {gains_str}")
            return True
        return False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def periodic(self):
        """Apply configuration to motor only when a gain value has changed."""
        return
        
        values_changed = False

        if self.use_slot0:
            values_changed |= self._check_slot("slot0", self.cfg.slot0)
        if self.use_slot1:
            values_changed |= self._check_slot("slot1", self.cfg.slot1)
        if self.use_slot2:
            values_changed |= self._check_slot("slot2", self.cfg.slot2)

        # 1.5 s debounce; FMS gate prevents field mishaps
        if values_changed:
            if wpilib.DriverStation.isFMSAttached():
                print(f"[{self.name}] FMS attached — gain updates disabled")
            else:
                now = wpilib.Timer.getFPGATimestamp()
                if now - self._last_apply_time >= 1.5:
                    self.talon_motor.configurator.apply(self.cfg, 0.050)
                    self._last_apply_time = now
                    print(f"[{self.name}] *** APPLIED GAINS TO MOTOR ***")

    def force_apply(self):
        """Bypass debounce and force-push current dashboard values to motor. Dev use only."""
        if wpilib.RobotBase.isSimulation() or wpilib.DriverStation.isFMSAttached():
            return
        for slot, slot_cfg in self._active_slots():
            new = _read_slot_nt(self.nt, slot)
            _apply_to_slot(slot_cfg, new)
            self.last_applied_values[slot] = dict(new)
        self.talon_motor.configurator.apply(self.cfg, 0.050)
        self._last_apply_time = wpilib.Timer.getFPGATimestamp()
        print(f"[{self.name}] FORCE APPLIED GAINS")

    def _active_slots(self):
        """Yield (slot_name, slot_cfg) for every enabled slot."""
        if self.use_slot0:
            yield "slot0", self.cfg.slot0
        if self.use_slot1:
            yield "slot1", self.cfg.slot1
        if self.use_slot2:
            yield "slot2", self.cfg.slot2

    def with_slot1(self) -> "EditablePID":
        self.use_slot1 = True
        return self

    def with_slot2(self) -> "EditablePID":
        self.use_slot2 = True
        return self
