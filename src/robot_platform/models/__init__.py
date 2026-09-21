"""Mechanical model profiles shared by viewing, audit and collision tools."""
from .urdf import ModelProfile, UrdfModel, discover_models

__all__ = ["ModelProfile", "UrdfModel", "discover_models"]
