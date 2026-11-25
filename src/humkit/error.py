"""Exceptions raised by humkit. Catch HumkitError to handle any of them."""


class HumkitError(Exception):
    """Base class for all humkit errors."""


class ModelLoadError(HumkitError):
    """A robot model could not be found, parsed or compiled."""


class StateDimensionError(HumkitError, ValueError):
    """An array had the wrong shape for this robot."""


class InvalidValueError(HumkitError, ValueError):
    """An array contained NaN or infinite values."""


class ControllerError(HumkitError):
    """A controller was misconfigured or produced invalid output."""


class ConfigError(HumkitError, ValueError):
    """A config file was malformed or had an invalid value."""
