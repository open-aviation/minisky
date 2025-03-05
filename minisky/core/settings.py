# %%
from pathlib import Path

import yaml

filename_settings = Path(__file__).parent.parent.parent / "settings.yml"

with open(filename_settings, "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

for key, value in data.items():
    globals()[key] = value


def data(path: str):
    return Path(__file__).parent.parent / "data" / path
