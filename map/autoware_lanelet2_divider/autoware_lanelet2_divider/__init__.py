import importlib.metadata as importlib_metadata

try:
    # This will read version from pyproject.toml
    __version__ = importlib_metadata.version(__package__ or __name__)
except importlib_metadata.PackageNotFoundError:
    __version__ = "development"
    
from .autoware_lanelet2_divider import AutowareLanlet2Divider
import autoware_lanelet2_divider.osmium_tool.osmium_tool as osmium_tool
import autoware_lanelet2_divider.xml_tool.xml_tool as xml_tool
import autoware_lanelet2_divider.data_preperation.data_preperation as data_preparation
from autoware_lanelet2_divider.debug import Debug, DebugMessageType