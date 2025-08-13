import importlib.metadata

try:
    __version__ = importlib.metadata.version("phospho_teleop")
except importlib.metadata.PackageNotFoundError:
    print("PackageNotFoundError: No package metadata was found for 'phospho_teleop'.")
    __version__ = "unknown"
