"""Typed success/error result values.

See: <https://doc.rust-lang.org/std/result/>
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Generic, Literal, Never, TypeAlias, TypeVar

ValueT_co = TypeVar("ValueT_co", covariant=True)
ErrorT_co = TypeVar("ErrorT_co", covariant=True)


@dataclass(frozen=True, slots=True)
class Ok(Generic[ValueT_co]):
    """A successful result containing a value."""

    _value: ValueT_co

    def ok(self) -> ValueT_co:
        """Return the success value."""
        return self._value

    def err(self) -> None:
        """Return no error value for a successful result."""
        return

    def is_ok(self) -> Literal[True]:
        """Return whether this result is successful."""
        return True

    def is_err(self) -> Literal[False]:
        """Return whether this result is an error."""
        return False

    def unwrap(self) -> ValueT_co:
        """Return the success value."""
        return self._value

    def unwrap_err(self) -> Never:
        """Raise because this result does not contain an error."""
        raise UnwrapError(
            self, f"called `Result.unwrap_err()` on successful result: {self._value!r}"
        )


@dataclass(frozen=True, slots=True)
class Err(Generic[ErrorT_co]):
    """An unsuccessful result containing an error value."""

    _value: ErrorT_co

    def ok(self) -> None:
        """Return no success value for an unsuccessful result."""
        return

    def err(self) -> ErrorT_co:
        """Return the error value."""
        return self._value

    def is_ok(self) -> Literal[False]:
        """Return whether this result is successful."""
        return False

    def is_err(self) -> Literal[True]:
        """Return whether this result is an error."""
        return True

    def unwrap(self) -> Never:
        """Raise because this result does not contain a success value."""
        raise UnwrapError(self, f"called `Result.unwrap()` on errored result: {self._value!r}")

    def unwrap_err(self) -> ErrorT_co:
        """Return the error value."""
        return self._value


class UnwrapError(Exception):
    """Raised when unwrapping the absent side of a result."""

    def __init__(self, result: Any, message: str) -> None:
        self.result = result
        super().__init__(message)


Result: TypeAlias = Ok[ValueT_co] | Err[ErrorT_co]
"""A value that is either successful (`Ok`) or unsuccessful (`Err`)."""
