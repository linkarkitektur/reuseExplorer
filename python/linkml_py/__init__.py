from __future__ import annotations

import logging



from ._core import *
from ._core import __doc__, __version__

# Define the logger format
log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(level=logging.INFO, format=log_format)

from .cli import run

__all__ = ["__doc__", "__version__", "speckle", "export", "run"]


