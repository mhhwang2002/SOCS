"""Phase-based DEVS with continuous evolution between transitions."""

from .DEVSph import DEVSph
from .cDEVS import cDEVS


class cDEVSph(DEVSph, cDEVS):
    """Combine declared phase transitions with delta_cont(elapsed)."""
