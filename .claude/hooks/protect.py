"""PreToolUse hook: never let Claude write the bridge's secret token or this machine's Python path."""
import json
import os
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path", "")
if os.path.basename(path) in (".bridge_token", ".local_python"):
    print(f"{os.path.basename(path)} is machine-local and must not be edited.", file=sys.stderr)
    sys.exit(2)
