"""PostToolUse hook: after Claude edits a .py file, compile it; a syntax error goes back to Claude (exit 2)."""
import json
import py_compile
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path", "")
if path.endswith(".py"):
    try:
        py_compile.compile(path, doraise=True)
    except py_compile.PyCompileError as e:
        print(f"Syntax error after the edit: {e.msg}", file=sys.stderr)
        sys.exit(2)
