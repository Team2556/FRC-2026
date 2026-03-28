from typing import Any, Generic, TypeVar, Union

import wpilib
from ntcore import NetworkTableInstance

T = TypeVar("T")

NTValue = Union[bool, int, float, str, list[bool], list[int], list[float], list[str]]


class NTEntry(Generic[T]):
    def __init__(self, entry, default: T):
        self._entry = entry
        self._default = default
        self._entry.set(default)

    def get(self) -> T:
        return self._entry.get(self._default)

    def set(self, value: T) -> None:
        self._entry.set(value)


class NTTable:
    def __init__(self, name: str, _table=None):
        self._table = _table or NetworkTableInstance.getDefault().getTable(name)
        self._entries: dict[str, NTEntry] = {}
        self._subtables: dict[str, "NTTable"] = {}
        self._sendables: dict[str, Any] = {}

    def get_subtable(self, name: str) -> "NTTable":
        if name not in self._subtables:
            self._subtables[name] = NTTable(name, self._table.getSubTable(name))
        return self._subtables[name]

    def _get_or_create(self, name: str, entry: NTEntry) -> NTEntry:
        if name not in self._entries:
            self._entries[name] = entry
        return self._entries[name]

    def bool(self, name: str, default: bool = False) -> NTEntry[bool]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getBooleanTopic(name).getEntry(default), default),
        )

    def int(self, name: str, default: int = 0) -> NTEntry[int]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getIntegerTopic(name).getEntry(default), default),
        )

    def float(self, name: str, default: float = 0.0) -> NTEntry[float]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getDoubleTopic(name).getEntry(default), default),
        )

    def string(self, name: str, default: str = "") -> NTEntry[str]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getStringTopic(name).getEntry(default), default),
        )

    def bool_array(self, name: str, default: list[bool] = []) -> NTEntry[list[bool]]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getBooleanArrayTopic(name).getEntry(default), default),
        )

    def int_array(self, name: str, default: list[int] = []) -> NTEntry[list[int]]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getIntegerArrayTopic(name).getEntry(default), default),
        )

    def float_array(self, name: str, default: list[float] = []) -> NTEntry[list[float]]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getDoubleArrayTopic(name).getEntry(default), default),
        )

    def string_array(self, name: str, default: list[str] = []) -> NTEntry[list[str]]:
        return self._get_or_create(
            name,
            NTEntry(self._table.getStringArrayTopic(name).getEntry(default), default),
        )

    def sendable(self, name: str, obj: Any) -> None:
        """Publish a Sendable object to this table path.

        Suitable for Field2d, Mechanism2d, SendableChooser, and any other
        wpiutil.Sendable.  The object is registered once; subsequent calls
        with the same name are no-ops so it is safe to call from __init__.

        The NT path will be  <table>/<name>/  (matching what Glass and
        Shuffleboard expect for structured sendables).

        Call ``update_sendables()`` from the owning subsystem's ``periodic()``
        so that mutable sendables (Field2d, Mechanism2d) push updated values
        to the network table every loop.
        """
        if name not in self._sendables:
            builder = wpilib.SendableBuilderImpl()
            builder.setTable(self._table.getSubTable(name))
            obj.initSendable(builder)   # populate properties without ownership transfer
            builder.startListeners()    # arm any NT input callbacks (e.g. SendableChooser)
            builder.update()            # publish initial values
            self._sendables[name] = (obj, builder)  # keep builder alive

    def update_sendables(self) -> None:
        """Push current Sendable values to NetworkTables.

        Call from the owning subsystem's ``periodic()`` to keep mutable
        sendables (Field2d, Mechanism2d) up to date.  Static sendables
        (SendableChooser options) are fine after the first update.
        """
        for _obj, builder in self._sendables.values():
            builder.update()

    def get(self, name: str) -> NTValue | None:
        entry = self._entries.get(name)
        return entry.get() if entry else None

    def set(self, name: str, value: NTValue) -> None:
        entry = self._entries.get(name)
        if entry:
            entry.set(value)
