#!/usr/bin/env python3
"""Dashboard web para controlar y monitorear main_test_l9110s.c por ROS 2.

Un solo archivo, sin dependencias fuera de ROS 2 (rclpy) y la libreria estandar.

  source /opt/ros/jazzy/setup.bash
  python3 micro_ros/tools/l9110s_dashboard.py            # http://127.0.0.1:8080
  python3 micro_ros/tools/l9110s_dashboard.py --host 0.0.0.0 --port 8080   # accesible desde la red

Requiere el micro-ROS agent corriendo y el firmware de test conectado. Topics:
  /prometeo/l9110s_test/estado  (String JSON)   <- se muestra
  /prometeo/l9110s_test/log     (String)        <- se muestra
  /prometeo/l9110s_test/cmd     (String)        -> botones

Con --host distinto de 127.0.0.1 CUALQUIERA en la red puede mover los motores.
"""
import argparse
import json
import queue
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NS = "/prometeo/l9110s_test"
CMD_OK = re.compile(
    r"^(stop|list|run( all| dirs| side| timeout| deadtime| cfg| nan| effort| stall [abcd])?|clear (all|[abcd]))$"
)

clientes = set()
clientes_lock = threading.Lock()
ultimo = {"estado": None, "log": []}   # para clientes que se conectan tarde


def difundir(tipo, dato):
    msg = f"event: {tipo}\ndata: {json.dumps(dato)}\n\n".encode()
    with clientes_lock:
        for q in list(clientes):
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass


class Puente(Node):
    def __init__(self):
        super().__init__("l9110s_dashboard")
        self.pub = self.create_publisher(String, f"{NS}/cmd", 10)
        self.create_subscription(String, f"{NS}/estado", self.on_estado, 10)
        self.create_subscription(String, f"{NS}/log", self.on_log, 50)

    def on_estado(self, m):
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        ultimo["estado"] = e
        difundir("estado", e)

    def on_log(self, m):
        ultimo["log"] = (ultimo["log"] + [m.data])[-200:]
        difundir("log", m.data)

    def enviar(self, texto):
        self.pub.publish(String(data=texto))


puente = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            body = HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue(maxsize=500)
            with clientes_lock:
                clientes.add(q)
            try:
                if ultimo["estado"]:
                    q.put(f"event: estado\ndata: {json.dumps(ultimo['estado'])}\n\n".encode())
                for l in ultimo["log"]:
                    q.put(f"event: log\ndata: {json.dumps(l)}\n\n".encode())
                while True:
                    try:
                        self.wfile.write(q.get(timeout=5))
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with clientes_lock:
                    clientes.discard(q)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/cmd":
            return self.send_error(404)
        n = min(int(self.headers.get("Content-Length", 0)), 64)
        cmd = self.rfile.read(n).decode(errors="ignore").strip().lower()
        ok = bool(CMD_OK.match(cmd))
        if ok:
            puente.enviar(cmd)
        self.send_response(200 if ok else 400)
        self.send_header("Content-Length", "0")
        self.end_headers()


HTML = r"""<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Test L9110S</title>
<style>
:root{--bg:#f5f6f8;--card:#fff;--ink:#1c2230;--mut:#6b7488;--line:#dfe3ea;--ok:#1a8f4c;--bad:#cc2f3d;--warn:#b7791f;--acc:#2f6fed;--run:#2f6fed}
@media (prefers-color-scheme:dark){:root{--bg:#12151c;--card:#1b2029;--ink:#e6e9f0;--mut:#8b94a8;--line:#2a3140;--ok:#3ec27a;--bad:#ff6b78;--warn:#e0a84a;--acc:#6b9cff;--run:#6b9cff}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{display:flex;gap:12px;align-items:center;padding:12px 16px;border-bottom:1px solid var(--line);background:var(--card);position:sticky;top:0;z-index:2;flex-wrap:wrap}
h1{font-size:16px;margin:0;flex:1}
.pill{padding:2px 10px;border-radius:99px;font-size:12px;border:1px solid var(--line);color:var(--mut)}
.pill.ok{color:var(--ok);border-color:var(--ok)}.pill.bad{color:var(--bad);border-color:var(--bad)}.pill.warn{color:var(--warn);border-color:var(--warn)}
main{display:grid;gap:12px;padding:12px 16px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));max-width:1200px;margin:auto}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
h2{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--mut);margin:0 0 10px}
.wide{grid-column:1/-1}
button{font:inherit;padding:7px 12px;border-radius:7px;border:1px solid var(--line);background:var(--card);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--acc)}button:disabled{opacity:.45;cursor:not-allowed}
button.primary{background:var(--acc);color:#fff;border-color:var(--acc)}
button.stop{background:var(--bad);color:#fff;border-color:var(--bad);font-weight:700;padding:10px 22px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.motor{display:grid;grid-template-columns:30px 1fr 70px;gap:8px;align-items:center;margin:8px 0}
.motor b{font-size:16px}
.bar{position:relative;height:16px;background:var(--bg);border:1px solid var(--line);border-radius:4px}
.bar::before{content:"";position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:var(--mut)}
.bar i{position:absolute;top:0;bottom:0;background:var(--acc);transition:all .15s}
.bar.stall i{background:var(--bad)}
.num{text-align:right;font-variant-numeric:tabular-nums}
.sub{color:var(--mut);font-size:12px}
table{width:100%;border-collapse:collapse}
td{padding:6px 4px;border-top:1px solid var(--line)}td:last-child{text-align:right}
.badge{display:inline-block;min-width:78px;text-align:center;padding:1px 8px;border-radius:5px;font-size:12px;font-weight:600;border:1px solid var(--line);color:var(--mut)}
.b-pass{color:var(--ok);border-color:var(--ok)}.b-fail{color:var(--bad);border-color:var(--bad)}
.b-parcial{color:var(--warn);border-color:var(--warn)}.b-corriendo{color:var(--run);border-color:var(--run);animation:p 1s infinite}
.b-abortado{color:var(--warn);border-color:var(--warn)}
@keyframes p{50%{opacity:.4}}
#log{height:300px;overflow:auto;background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:8px;font:12px/1.5 ui-monospace,monospace;white-space:pre-wrap}
.l-PASS{color:var(--ok)}.l-FAIL{color:var(--bad)}.l-INFO{color:var(--ink)}
.warnbox{background:color-mix(in srgb,var(--warn) 15%,transparent);border:1px solid var(--warn);border-radius:8px;padding:8px 12px;margin:0 16px;max-width:1168px}
</style></head><body>
<header>
  <h1>Test driver L9110S</h1>
  <span class="pill" id="p-link">sin datos</span>
  <span class="pill" id="p-test">inactivo</span>
  <button class="stop" onclick="cmd('stop')" title="Esc o Espacio">■ STOP</button>
</header>
<div class="warnbox">Levantar las ruedas del piso antes de correr. STOP: <kbd>Esc</kbd> o <kbd>Espacio</kbd>.</div>
<main>
  <section>
    <h2>Motores</h2>
    <div id="motores"></div>
    <div class="row" style="margin-top:10px">
      <span class="pill" id="p-timeout">timeout: —</span>
      <button onclick="cmd('clear all')">Liberar atascos</button>
    </div>
  </section>
  <section>
    <h2>Tests</h2>
    <div class="row" style="margin-bottom:8px">
      <button class="primary" onclick="cmd('run all')">▶ Correr todos</button>
      <button onclick="cmd('list')">Listar</button>
    </div>
    <table id="tests"></table>
    <h2 style="margin-top:14px">Stall (manual)</h2>
    <div class="sub" style="margin-bottom:6px">Elegir motor, bloquear la rueda con la mano; arranca a los 5 s.</div>
    <div class="row" id="stall"></div>
  </section>
  <section class="wide">
    <h2>Log <button style="float:right;padding:2px 8px;font-size:12px" onclick="document.getElementById('log').textContent=''">limpiar</button></h2>
    <div id="log"></div>
  </section>
</main>
<script>
const NOM=['A','B','C','D'], DESC=['delantero izq','trasero izq','delantero der','trasero der'];
const TESTS=[['boot','Arranque (automático)',0],['dirs','Giro de cada motor',1],['side','Lados por separado',1],['timeout','Watchdog de órdenes',1],
             ['deadtime','Pausa al invertir',1],['cfg','Config por motor',1],['nan','NaN / inf frena',1],['effort','Esfuerzo maximo',1],['stall','Atasco (manual)',0]];
const $=id=>document.getElementById(id);
function cmd(c){fetch('/cmd',{method:'POST',body:c}).then(r=>{if(!r.ok)add('[FAIL] comando rechazado: '+c)})}
document.addEventListener('keydown',e=>{if(e.key==='Escape'||(e.key===' '&&e.target===document.body)){e.preventDefault();cmd('stop')}});

$('motores').innerHTML=NOM.map((n,i)=>`<div class="motor"><b>${n}</b><div><div class="bar" id="bar${i}"><i></i></div>
  <div class="sub">${DESC[i]} · <span id="pul${i}">0 pulsos</span> · <span id="rate${i}">0/s</span></div></div><div class="num" id="d${i}">0.00</div></div>`).join('');
$('tests').innerHTML=TESTS.map(([k,t,run])=>`<tr><td>${t}${run?` <button style="padding:1px 8px;font-size:12px" onclick="cmd('run ${k}')">▶</button>`:''}</td><td><span class="badge" id="r-${k}">pendiente</span></td></tr>`).join('');
$('stall').innerHTML=NOM.map(n=>`<button onclick="cmd('run stall ${n.toLowerCase()}')">Motor ${n}</button>`).join('');

let prev=null, ult=0;
function set(id,txt,cls){const e=$(id);e.textContent=txt;e.className=cls}
function onEstado(e){
  ult=Date.now();
  const now=performance.now();
  (e.duties||[]).forEach((d,i)=>{
    const bar=$('bar'+i), fill=bar.firstElementChild, stall=e.fallas.stall[i];
    const w=Math.min(Math.abs(d),1)*50;
    fill.style.left=d>=0?'50%':(50-w)+'%'; fill.style.width=w+'%';
    bar.className='bar'+(stall?' stall':'');
    $('d'+i).textContent=(d>=0?'+':'')+d.toFixed(2);
    $('pul'+i).textContent=e.pulsos[i]+' pulsos'+(stall?' · ATASCADO':'');
    if(prev&&now>prev.t){const dp=(e.pulsos[i]-prev.p[i])>>>0; $('rate'+i).textContent=(dp*1000/(now-prev.t)).toFixed(0)+'/s'}
  });
  prev={t:now,p:e.pulsos};
  set('p-timeout','timeout: '+(e.fallas.timeout?'ACTIVO (frenado)':'ok'),'pill '+(e.fallas.timeout?'warn':'ok'));
  set('p-test',e.test?('corriendo: '+e.test):'inactivo','pill '+(e.test?'ok':''));
  for(const [k] of TESTS){const r=e.resultados[k]||'pendiente';set('r-'+k,r,'badge b-'+r)}
}
function add(l){
  const m=l.match(/^\[(\w+)\]/), cls=m?'l-'+m[1]:'l-INFO';
  const box=$('log'), pegado=box.scrollTop+box.clientHeight>=box.scrollHeight-20;
  const s=document.createElement('div'); s.className=cls; s.textContent=new Date().toLocaleTimeString()+' '+l; box.appendChild(s);
  while(box.childElementCount>500) box.removeChild(box.firstChild);
  if(pegado) box.scrollTop=box.scrollHeight;
}
const es=new EventSource('/events');
es.addEventListener('estado',m=>onEstado(JSON.parse(m.data)));
es.addEventListener('log',m=>add(JSON.parse(m.data)));
es.onerror=()=>set('p-link','dashboard desconectado','pill bad');
setInterval(()=>{ if(es.readyState!==1)return;
  const ok=Date.now()-ult<1500; set('p-link',ok?'robot conectado':'sin datos del robot','pill '+(ok?'ok':'warn')); },500);
</script></body></html>
"""


def main():
    global puente
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()

    rclpy.init()
    puente = Puente()
    threading.Thread(target=rclpy.spin, args=(puente,), daemon=True).start()

    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    print(f"Dashboard en http://{a.host}:{a.port}  (Ctrl+C para salir)")
    if a.host != "127.0.0.1":
        print("ATENCION: accesible desde la red; cualquiera puede mover los motores")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        puente.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
