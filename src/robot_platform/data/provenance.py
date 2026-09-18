"""Carry simulation provenance across legacy recording adapters."""
import json
import os


def write_provenance(file, provenance=None):
    if provenance is None:
        settings = json.loads(os.environ.get("ROBOT_PLATFORM_SETTINGS_JSON", "{}"))
        provenance = settings.get("simulation")
    if provenance is not None:
        if not isinstance(provenance, dict) or not isinstance(provenance.get("run_id"), str) or provenance.get("evidence") != "synthetic":
            raise ValueError("invalid synthetic episode provenance")
        file.attrs["data_source"] = "simulation"
        file.attrs["simulation_provenance"] = json.dumps(provenance, sort_keys=True)
    else:
        file.attrs["data_source"] = "unspecified"
