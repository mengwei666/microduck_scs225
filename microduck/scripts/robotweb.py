#!/usr/bin/env python3
"""Minimal web console for robotd — control the robot from a phone over WiFi.

Run on the board:
    python3 /root/robotweb.py
then open http://<board-ip>:8080 on a phone or laptop on the same network.

It speaks the same NDJSON JSON-RPC to /run/robotd.sock that robotctl does, so
there is no second API to keep in step. Bind is 0.0.0.0 and there is no login:
this is a bench/LAN tool, do not expose it to the internet.
"""

import json
import socket
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOCK = "/run/robotd.sock"
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080


def rpc(method, params=None, notify=False):
    msg = {"jsonrpc": "2.0", "method": method}
    if params is not None:
        msg["params"] = params
    if not notify:
        msg["id"] = 1
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect(SOCK)
    s.sendall((json.dumps(msg) + "\n").encode())
    if notify:
        s.close()
        return None
    buf = b""
    while b"\n" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
    s.close()
    return json.loads(buf.decode().splitlines()[0]) if buf else None


PAGE = """<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Microduck 控制台</title>
<style>
 body{font-family:system-ui,sans-serif;margin:16px;background:#111;color:#eee}
 h1{font-size:18px;margin:0 0 12px}
 .row{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:10px}
 button{flex:1 1 40%;min-height:56px;font-size:16px;border:0;border-radius:10px;
        background:#2d6cdf;color:#fff;padding:10px}
 button.gray{background:#444} button.red{background:#b3261e}
 button.green{background:#1e7d3c}
 #log{white-space:pre-wrap;background:#000;padding:8px;border-radius:8px;
      min-height:90px;font-size:12px;color:#9f9}
 input[type=range]{width:100%}
</style></head><body>
<h1>Microduck 控制台 <span id="net" style="font-size:12px;color:#888"></span></h1>
<div class="row">
  <button class="green" onclick="cmd('init')">上电回中位</button>
  <button onclick="cmd('enable')">使能策略</button>
  <button class="gray" onclick="cmd('disable')">关闭策略</button>
  <button class="red" onclick="cmd('relax')">卸力</button>
</div>
<div class="row">
  <button onpointerdown="hold(1)" onpointerup="release()" onpointerleave="release()">▲ 前进</button>
  <button onpointerdown="hold(-1)" onpointerup="release()" onpointerleave="release()">▼ 后退</button>
  <button onpointerdown="turn(1)" onpointerup="release()" onpointerleave="release()">↶ 左转</button>
  <button onpointerdown="turn(-1)" onpointerup="release()" onpointerleave="release()">↷ 右转</button>
</div>
<div class="row">
  <button class="red" onclick="cmd('stop')">■ 停止</button>
  <button class="gray" onclick="cmd('do','kick_left')">踢左</button>
  <button class="gray" onclick="cmd('do','roulade')">前滚翻</button>
</div>
<div class="row"><div style="flex:1 1 100%">
  速度 <span id="spd">0.20</span> m/s
  <input type="range" min="0.05" max="0.6" step="0.05" value="0.2"
         oninput="document.getElementById('spd').textContent=(+this.value).toFixed(2); speed=+this.value">
</div></div>
<pre id="log"></pre>
<script>
let speed=0.2, timer=null;
const log=document.getElementById('log');
function show(x){log.textContent=(typeof x==='string'?x:JSON.stringify(x,null,2));}
async function post(body){
  const r=await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
                            body:JSON.stringify(body)});
  show(await r.text());
}
function cmd(name,arg){post({cmd:name,arg:arg});}
function send(vx,vy,vyaw){fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({cmd:'move',vx,vy,vyaw})}).catch(()=>{});}
function hold(sign){release(); send(sign*speed,0,0); timer=setInterval(()=>send(sign*speed,0,0),100);}
function turn(sign){release(); send(0,0,sign*0.5); timer=setInterval(()=>send(0,0,sign*0.5),100);}
function release(){if(timer){clearInterval(timer);timer=null;} send(0,0,0);}
document.getElementById('net').textContent=location.host;
setInterval(async()=>{try{const r=await fetch('/status');show(await r.text());}catch(e){}},3000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/status":
            try:
                self._send(200, json.dumps(rpc("robot.health"), ensure_ascii=False))
            except Exception as e:
                self._send(200, json.dumps({"error": str(e)}))
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path != "/cmd":
            self._send(404, "not found", "text/plain")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except Exception as e:
            self._send(400, json.dumps({"error": str(e)}))
            return
        try:
            c = req.get("cmd")
            if c == "init":
                out = rpc("robot.init")
            elif c == "relax":
                out = rpc("robot.relax")
            elif c == "stop":
                out = rpc("robot.stop")
            elif c == "enable":
                out = rpc("robot.enable", {"on": True})
            elif c == "disable":
                out = rpc("robot.enable", {"on": False})
            elif c == "do":
                out = rpc("robot.do", {"skill": req.get("arg")})
            elif c == "move":
                rpc("robot.move", {"vx": float(req.get("vx", 0)),
                                   "vy": float(req.get("vy", 0)),
                                   "vyaw": float(req.get("vyaw", 0))},
                    notify=True)
                out = {"sent": True}
            else:
                out = {"error": "unknown cmd"}
            self._send(200, json.dumps(out, ensure_ascii=False))
        except Exception as e:
            self._send(500, json.dumps({"error": str(e)}))

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    print(f"serving on http://0.0.0.0:{PORT}/  (robotd socket {SOCK})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
