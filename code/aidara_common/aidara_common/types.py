"""Common types collection."""

from enum import Enum


class RobotName(Enum):
    """Enum class with valid robot names."""

    VBOT = "vbot"
    PEPPER = "pepper"

    def __str__(self) -> str:
        """Format the enum variant."""
        return self.value

    @classmethod
    def get_valid_options(cls) -> list[str]:
        """Return all valid options."""
        return [option.value for option in cls.__members__.values()]
