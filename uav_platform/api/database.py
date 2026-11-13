"""
database.py — Structured JSON persistence for simulation runs, experiments, and history.
"""
import os
import json
import threading
import uuid
from datetime import datetime
from typing import Dict, List, Any, Optional

DB_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "data", "experiments_db.json")

class Database:
    def __init__(self, path: str = DB_PATH):
        self.path = path
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        if not os.path.exists(self.path):
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump({"experiments": []}, f, indent=2)

    def _read(self) -> Dict[str, Any]:
        with self._lock:
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                return {"experiments": []}

    def _write(self, data: Dict[str, Any]):
        with self._lock:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)

    def save_experiment(self, exp: Dict[str, Any]) -> str:
        """Saves or updates an experiment record. Returns the record id."""
        data = self._read()
        exp_id = exp.get("id") or f"exp_{uuid.uuid4().hex[:8]}"
        exp["id"] = exp_id
        if "created_at" not in exp:
            exp["created_at"] = datetime.utcnow().isoformat() + "Z"

        existing = False
        for i, item in enumerate(data["experiments"]):
            if item.get("id") == exp_id:
                data["experiments"][i] = exp
                existing = True
                break
        if not existing:
            data["experiments"].insert(0, exp)

        self._write(data)
        return exp_id

    def get_experiment(self, exp_id: str) -> Optional[Dict[str, Any]]:
        data = self._read()
        for item in data["experiments"]:
            if item.get("id") == exp_id:
                return item
        return None

    def list_experiments(self, limit: int = 50, config_name: Optional[str] = None) -> List[Dict[str, Any]]:
        data = self._read()
        results = data.get("experiments", [])
        if config_name:
            results = [r for r in results if r.get("config_name") == config_name]
        return results[:limit]

    def delete_experiment(self, exp_id: str) -> bool:
        data = self._read()
        initial_len = len(data["experiments"])
        data["experiments"] = [r for r in data["experiments"] if r.get("id") != exp_id]
        if len(data["experiments"]) < initial_len:
            self._write(data)
            return True
        return False

    def get_stats(self) -> Dict[str, Any]:
        data = self._read()
        exps = data.get("experiments", [])
        completed = [e for e in exps if e.get("status") == "completed"]
        baseline_count = sum(1 for e in exps if e.get("config_name") == "baseline")
        tuned_count = sum(1 for e in exps if e.get("config_name") == "tuned")

        return {
            "total_experiments": len(exps),
            "completed_runs": len(completed),
            "baseline_runs": baseline_count,
            "tuned_runs": tuned_count,
            "latest_run": exps[0] if exps else None
        }

db = Database()

