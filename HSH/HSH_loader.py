import json
from typing import Dict

def load_instance_from_json(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)