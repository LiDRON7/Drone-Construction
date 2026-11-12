# Sends a command to the Fusion360MCP add-in (Fusion must be open, add-in running).
# usage: python3 run_in_fusion.py execute_code -code propeller_guard.py   |   python3 run_in_fusion.py ping
import sys, json, socket, os
cmd = sys.argv[1]
p = ({"code": f"HERE = {os.path.dirname(os.path.abspath(sys.argv[3]))!r}\n" + open(sys.argv[3]).read()}  # scripts save relative to their own folder
     if sys.argv[2:3] == ['-code'] else json.loads(sys.argv[2]) if len(sys.argv) > 2 else {})
s = socket.create_connection(("localhost", 9876), timeout=120)
s.sendall((json.dumps({"type": cmd, "params": p}) + "\n").encode())
buf = b""
while not buf.endswith(b"\n"):
    chunk = s.recv(65536)
    if not chunk: break
    buf += chunk
r = json.loads(buf)
print(json.dumps(r, indent=1, default=str)[:6000])
sys.exit(r.get("status") == "error")
