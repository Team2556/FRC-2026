from ntcore import NetworkTableInstance
from typing import Generic, TypeVar, Union

T = TypeVar("T")

NTValue = Union[bool, int, float, str, list[bool], list[int], list[float], list[str]]


class NTEntry(Generic[T]):
    def __init__(self, publisher, subscriber, default: T):
        self._publisher = publisher
        self._subscriber = subscriber
        self._default = default
        self._publisher.set(default)

    def get(self) -> T:
        return self._subscriber.get(self._default)

    def set(self, value: T) -> None:
        self._publisher.set(value)


class NTTable:
    def __init__(self, name: str, _table=None):
        self._table = _table or NetworkTableInstance.getDefault().getTable(name)
        self._entries: dict[str, NTEntry] = {}
        self._subtables: dict[str, "NTTable"] = {}

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
            NTEntry(
                self._table.getBooleanTopic(name).publish(),
                self._table.getBooleanTopic(name).subscribe(default),
                default,
            ),
        )

    def int(self, name: str, default: int = 0) -> NTEntry[int]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getIntegerTopic(name).publish(),
                self._table.getIntegerTopic(name).subscribe(default),
                default,
            ),
        )

    def float(self, name: str, default: float = 0.0) -> NTEntry[float]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getDoubleTopic(name).publish(),
                self._table.getDoubleTopic(name).subscribe(default),
                default,
            ),
        )

    def string(self, name: str, default: str = "") -> NTEntry[str]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getStringTopic(name).publish(),
                self._table.getStringTopic(name).subscribe(default),
                default,
            ),
        )

    def bool_array(self, name: str, default: list[bool] = []) -> NTEntry[list[bool]]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getBooleanArrayTopic(name).publish(),
                self._table.getBooleanArrayTopic(name).subscribe(default),
                default,
            ),
        )

    def int_array(self, name: str, default: list[int] = []) -> NTEntry[list[int]]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getIntegerArrayTopic(name).publish(),
                self._table.getIntegerArrayTopic(name).subscribe(default),
                default,
            ),
        )

    def float_array(self, name: str, default: list[float] = []) -> NTEntry[list[float]]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getDoubleArrayTopic(name).publish(),
                self._table.getDoubleArrayTopic(name).subscribe(default),
                default,
            ),
        )

    def string_array(self, name: str, default: list[str] = []) -> NTEntry[list[str]]:
        return self._get_or_create(
            name,
            NTEntry(
                self._table.getStringArrayTopic(name).publish(),
                self._table.getStringArrayTopic(name).subscribe(default),
                default,
            ),
        )

    def get(self, name: str) -> NTValue | None:
        entry = self._entries.get(name)
        return entry.get() if entry else None

    def set(self, name: str, value: NTValue) -> None:
        entry = self._entries.get(name)
        if entry:
            entry.set(value)