import serial,time,threading,json
from pathlib import Path
import numpy as np,sounddevice as sd
from scipy.io import wavfile
root=Path('companion/sweeps/lightsaber-solo-check-20261109');root.mkdir(exist_ok=True)
s=serial.Serial('COM6',115200,timeout=.4);time.sleep(2);s.write(b'l');time.sleep(4);s.reset_input_buffer()
blocks=[]
def cb(y,n,t,status):blocks.append((time.perf_counter()+t.inputBufferAdcTime-t.currentTime,y[:,0].copy(),str(status)))
dev=next(i for i,d in enumerate(sd.query_devices()) if 'Mad Catz' in d['name'] and d['hostapi']==2)
with sd.InputStream(device=dev,channels=1,samplerate=48000,blocksize=480,latency='low',callback=cb):
 for mode in ('U','V','W','A','B','C'):
  s.write(b'l');time.sleep(.1);blocks.clear();s.write(mode.encode());deadline=time.time()+10;begin=None;raw=None;log=[]
  while time.time()<deadline:
   line=s.readline().decode(errors='replace').strip();log.append(line)
   if line.startswith('$capture_begin'):begin=time.perf_counter()
   if line.startswith('$capture_data'):
    _,n,scale=line.split(',');n=int(n);raw=s.read(n*10)
    if len(raw)!=n*10:raise RuntimeError('short data')
   if line.startswith('$mix_stats'):print(line,flush=True);break
  if raw is None or begin is None:raise RuntimeError('capture timeout: '+str(log[-10:]))
  time.sleep(.2);record=list(blocks);audio=np.concatenate([x[1] for x in record]);times=np.concatenate([x[0]+np.arange(len(x[1]))/48000 for x in record])-begin
  arr=np.frombuffer(raw,dtype=[('t','<u4'),('a','<i2',(3,))]);np.savez_compressed(root/(mode+'.npz'),t=arr['t'],a=arr['a'],audio_t0=times[0],scale_mg=float(scale));wavfile.write(root/(mode+'.wav'),48000,audio)
  x=arr['a'].astype(float)*float(scale)/1000;clip=int(np.sum(abs(arr['a'].astype(int))>=32760));y=audio[(times>.1)&(times<2.9)]
  print(mode,'samples',n,'imu RMS g',np.sqrt(np.var(x,axis=0).sum()),'imu clip',clip,'mic peak',np.max(abs(y)),'mic rms',np.sqrt(np.mean(y*y)),'status',set(x[2] for x in record),flush=True)
  (root/(mode+'.log')).write_text('\n'.join(log),encoding='utf-8');s.write(b't');time.sleep(1)
s.close()
