from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from util.nt_util import NTTable


class EditablePID:
    def __init__(self, name: str, talon_motor: TalonFX, cfg: TalonFXConfiguration, use_slot0: bool = True, use_slot1: bool = False, use_slot2: bool = False):
        self.name = name
        self.talon_motor = talon_motor
        self.cfg = cfg

        self.use_slot0 = use_slot0
        self.use_slot1 = use_slot1
        self.use_slot2 = use_slot2

        self.nt = NTTable(self.name).get_subtable("EditablePID")

        if self.use_slot0:
            self.nt.float("slot0/k_p", self.cfg.slot0.k_p)
            self.nt.float("slot0/k_i", self.cfg.slot0.k_i)
            self.nt.float("slot0/k_d", self.cfg.slot0.k_d)
        if self.use_slot1:
            self.nt.float("slot1/k_p", self.cfg.slot1.k_p)
            self.nt.float("slot1/k_i", self.cfg.slot1.k_i)
            self.nt.float("slot1/k_d", self.cfg.slot1.k_d)
        if self.use_slot2:
            self.nt.float("slot2/k_p", self.cfg.slot2.k_p)
            self.nt.float("slot2/k_i", self.cfg.slot2.k_i)
            self.nt.float("slot2/k_d", self.cfg.slot2.k_d)

    def periodic(self):
        value_changed = False

        if self.use_slot0:
            value_changed = (
                self.cfg.slot0.k_p != self.nt.get("slot0/k_p")
                or self.cfg.slot0.k_i != self.nt.get("slot0/k_i")
                or self.cfg.slot0.k_d != self.nt.get("slot0/k_d")
            )
        if self.use_slot1:
            value_changed = value_changed or (
                self.cfg.slot1.k_p != self.nt.get("slot1/k_p")
                or self.cfg.slot1.k_i != self.nt.get("slot1/k_i")
                or self.cfg.slot1.k_d != self.nt.get("slot1/k_d")
            )
        if self.use_slot2:
            value_changed = value_changed or (
                self.cfg.slot2.k_p != self.nt.get("slot2/k_p")
                or self.cfg.slot2.k_i != self.nt.get("slot2/k_i")
                or self.cfg.slot2.k_d != self.nt.get("slot2/k_d")
            )

        if value_changed:
            if self.use_slot0:
                self.cfg.slot0.k_p = self.nt.get("slot0/k_p")
                self.cfg.slot0.k_i = self.nt.get("slot0/k_i")
                self.cfg.slot0.k_d = self.nt.get("slot0/k_d")
            if self.use_slot1:
                self.cfg.slot1.k_p = self.nt.get("slot1/k_p")
                self.cfg.slot1.k_i = self.nt.get("slot1/k_i")
                self.cfg.slot1.k_d = self.nt.get("slot1/k_d")
            if self.use_slot2:
                self.cfg.slot2.k_p = self.nt.get("slot2/k_p")
                self.cfg.slot2.k_i = self.nt.get("slot2/k_i")
                self.cfg.slot2.k_d = self.nt.get("slot2/k_d")
            self.talon_motor.configurator.apply(self.cfg)

    def with_slot1(self) -> "EditablePID":
        self.use_slot1 = True
        return self

    def with_slot2(self) -> "EditablePID":
        self.use_slot2 = True
        return self