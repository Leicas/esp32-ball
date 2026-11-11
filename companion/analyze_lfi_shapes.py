from pathlib import Path
import csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('companion/sweeps/lfi-centered-sides-20261109');r=list(csv.DictReader((p/'measurements.csv').open()));r=[{k: (v if k in ('id','phase','audio_status') else float(v)) for k,v in x.items()} for x in r if x['mask']=='1']
fig,ax=plt.subplots(3,1,figsize=(11,12),constrained_layout=True)
for code,name,color in [(16,'Centered','#6366f1'),(32,'Positive only','#e67e22'),(33,'Negative only','#16a085')]:
 for amp,style in [(.06,':'),(.12,'--'),(.25,'-.'),(.5,'-')]:
  d=sorted([x for x in r if x['invert']==code and x['amp']==amp],key=lambda x:x['hz']);f=[x['hz'] for x in d]
  if not d:continue
  ax[0].plot(f,[x['imu_rms_mg'] for x in d],style,color=color,label=f'{name} {amp:.0%}')
  ax[1].plot(f,[100*np.hypot(x['imu_h2_mg'],x['imu_h3_mg'])/max(x['imu_h1_mg'],1e-6) for x in d],style,color=color)
  ax[2].plot(f,[x['imu_h1_mg']/(amp*(1 if code==16 else .5)) for x in d],style,color=color)
for a in ax:a.grid(alpha=.2);a.set_xlabel('Hz')
ax[0].set(ylabel='Total acceleration RMS (mg)',title='LFi: centered and smooth side-biased sine drive');ax[0].legend(ncol=3,fontsize=8)
ax[1].set(ylabel='2nd + 3rd harmonic / fundamental (%)',ylim=(0,150),title='Low-order nonlinearity indicator; weak fundamentals are noise-sensitive')
ax[2].set(ylabel='Fundamental mg / input AC peak',title='Normalized sensitivity: growth with level indicates nonlinearity')
fig.savefig(p/'comparison.png',dpi=150)
for code in (16,32,33):
 print('SHAPE',code)
 for amp in (.06,.12,.25,.5):
  d=[x for x in r if x['invert']==code and x['amp']==amp];print(amp,[(int(x['hz']),round(x['imu_rms_mg']),round(100*np.hypot(x['imu_h2_mg'],x['imu_h3_mg'])/max(x['imu_h1_mg'],1e-6))) for x in d])
