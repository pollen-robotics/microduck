"""Subscribe to robotd's robot.map and dump frames (also saves one for rendering)."""
import base64, json, socket, sys, time
import numpy as np

n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
out = sys.argv[2] if len(sys.argv) > 2 else None

s = socket.socket(socket.AF_UNIX); s.settimeout(20); s.connect("/tmp/dsm/a.sock")
f = s.makefile("rw")
f.write(json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.map", "params": {}}) + "\n")
f.flush()
print("subscribe ->", f.readline().strip())

got = 0
last = None
while got < n:
    m = json.loads(f.readline())
    if m.get("method") != "map.frame":
        continue
    p = m["params"]
    cells = np.frombuffer(base64.b64decode(p["cells"]), dtype=np.uint8)
    got += 1
    last = p
    print(f"seq={p['seq']} {p['rows']}x{p['cols']} @{p['cell_m']} "
          f"origin=({p['x_min']:.2f},{p['y_min']:.2f}) "
          f"pose=({p['x']:+.2f},{p['y']:+.2f},{p['yaw']:+.2f}) "
          f"tracking={p['tracking']} submaps={p['n_submaps']} loops={p['n_loops']} "
          f"windows={p.get('windows')} still={p.get('still')} "
          f"free={int((cells==1).sum())} wall={int((cells==2).sum())} "
          f"unknown={int((cells==0).sum())} bytes={len(base64.b64decode(p['cells']))}")

if out and last:
    json.dump({"frame": last, "trail": []}, open(out, "w"))
    print("wrote", out)
