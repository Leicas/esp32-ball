"""Prepare embedded 16 kHz PCM from the user's local reference recordings.
Run after ffmpeg has decoded the supplied MP3s to mono 16 kHz WAV files.
No downloads; source recordings are not distributed by this script.
"""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.io import wavfile
from scipy.signal import butter,sosfilt
from scipy.ndimage import uniform_filter1d

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/'build/lightsaber-reference'
def load(name):
    sr,x=wavfile.read(WORK/name)
    assert sr==16000 and x.ndim==1
    return x.astype(float)/32768

def clip(x,a,b,loop=False):
    x=x[round(a*16000):round(b*16000)].copy()
    x=sosfilt(butter(2,25,fs=16000,btype='highpass',output='sos'),x)
    if loop:
        n=19200;w=np.linspace(0,1,n,endpoint=False)
        seam=x[-n:]*np.cos(w*np.pi/2)+x[:n]*np.sin(w*np.pi/2)
        x=np.r_[x[n:-n],seam]
        # Flatten slow recording/overlap swells without changing the waveform.
        rms=np.sqrt(uniform_filter1d(x*x,2400,mode='wrap')+1e-9)
        x*=np.clip(np.median(rms)/rms,.5,2)
    else:
        n=min(160,len(x)//10);x[:n]*=np.linspace(0,1,n);x[-n:]*=np.linspace(1,0,n)
    x*=.94/max(abs(x).max(),1e-9)
    return np.rint(x*32767).astype(np.int16)

hum=load('reference.wav');swings=load('swings.wav')
spec=[('ignition',hum,.60,1.85,False),('idle',hum,2.4,7.4,True),
      ('swing0',swings,.12,.71,False),('swing1',swings,1.40,1.99,False)]
out=['#pragma once','#include <stdint.h>','namespace LightsaberSamples {','constexpr int kRate=16000;']
manifest=[]
for name,x,a,b,loop in spec:
    y=clip(x,a,b,loop)
    if name.startswith('swing'):
        # Remove bass rumble and leave bandwidth for up to 2.2x playback.
        z=sosfilt(butter(3,[180,3000],fs=16000,btype='bandpass',output='sos'),y.astype(float))
        z*=.94*32767/max(abs(z).max(),1)
        y=np.rint(z).astype(np.int16)
    wavfile.write(WORK/(name+'.wav'),16000,y)
    out.append(f'inline constexpr int16_t {name}[]={{')
    out.extend(','.join(map(str,y[i:i+24]))+',' for i in range(0,len(y),24))
    out.append('};')
    manifest.append(dict(name=name,start=a,end=b,loop=loop,samples=len(y),sha256=hashlib.sha256(y.tobytes()).hexdigest()))
out+=['struct Clip { const int16_t *data; uint32_t count; };',
      'inline constexpr Clip swings[]={'+','.join('{'+f'swing{i},sizeof(swing{i})/sizeof(int16_t)'+'}' for i in range(2))+'};','}']
(ROOT/'src/lightsaber_samples.h').write_text('\n'.join(out)+'\n')
(WORK/'manifest.json').write_text(json.dumps(manifest,indent=2))
print('PCM bytes:',sum(s['samples']*2 for s in manifest))
