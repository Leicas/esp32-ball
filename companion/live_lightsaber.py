"""Local live lightsaber controls. Keeps one USB connection open."""
import json,time,threading,re
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import serial
port=serial.Serial('COM6',115200,timeout=.2)
state={'connected':True,'levels':[100,60,100,100],'ack':None,'status':'Connecting','last_seen':time.time()}
lock=threading.Lock()
def reader():
 try:
  while True:
   line=port.readline().decode(errors='replace').strip()
   if not line:continue
   with lock:
    state['last_seen']=time.time()
    if line.startswith('# mixer='):
     m=re.search(r'mixer=(\d+),(\d+),(\d+),(\d+) startup=(\d+)',line)
     if m:state['levels']=[int(x) for x in m.groups()[:4]];state['ack']=line
    if 'mode=' in line:state['status']=line
 except Exception as e:
  with lock:state['connected']=False;state['status']=str(e)
threading.Thread(target=reader,daemon=True).start()
def send(levels,restart=False):
 with lock:
  port.write(('Z'+' '.join(map(str,levels))+f' {int(restart)}\n').encode());port.flush()
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*a):pass
 def respond(self,data,kind='application/json',code=200):
  self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
 def do_GET(self):
  if self.path=='/':self.respond(Path(__file__).with_suffix('.html').read_bytes(),'text/html; charset=utf-8')
  elif self.path=='/state':
   with lock:s=dict(state)
   s['connected']=s['connected'] and time.time()-s['last_seen']<8;self.respond(json.dumps(s).encode())
  else:self.respond(b'{}',code=404)
 def do_POST(self):
  if self.path!='/mix':return self.respond(b'{}',code=404)
  if self.headers.get('Origin') not in (None,'http://127.0.0.1:8767','http://localhost:8767'):return self.respond(b'{}',code=403)
  try:
   n=int(self.headers.get('Content-Length','0'))
   if n>512:raise ValueError('request too large')
   d=json.loads(self.rfile.read(n));v=d['levels']
   if len(v)!=4 or any(type(x)!=int or x<0 or x>100 for x in v):raise ValueError('levels must be 0–100')
   send(v,bool(d.get('startup',False)))
   if d.get('mode') in ('textured','rolling','sliding','rain','marbles'):
    with lock:port.write({'textured':b't','rolling':b'r','sliding':b's','rain':b'y','marbles':b'm'}[d['mode']])
   self.respond(b'{"sent":true}')
  except Exception as e:self.respond(json.dumps({'error':str(e)}).encode(),code=400)
time.sleep(4);send([100,60,100,100])
print('LIVE http://127.0.0.1:8767',flush=True)
ThreadingHTTPServer(('127.0.0.1',8767),Handler).serve_forever()
