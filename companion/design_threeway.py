"""Measured, equal-delay three-output FIR design. Requires numpy/scipy/matplotlib.

Pair sums identify only the REAL vector Gram matrix, not complex phase.
Consequently every path uses a symmetric FIR with the SAME delay. The model
is local to the measured drive level; predictions are not validation.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.linalg import eigh
from scipy.ndimage import gaussian_filter1d
from scipy.signal import firwin2, freqz
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

CHANNELS = ('1', '3', '4')
NAMES = ('LFi', 'HF', 'LF')
FS = 2000
TAPS = 65

def load_model(path):
    rows = list(csv.DictReader(open(path, newline='')))
    modes = {m: {float(r['hz']): r for r in rows if r['mode'] == m and float(r['hz']) > 0}
             for m in dict.fromkeys(r['mode'] for r in rows)}
    required = list(CHANNELS) + [f'{a}{sign}{b}' for a,b in [('1','3'),('1','4'),('3','4')] for sign in ('+','-')]
    if any(m not in modes for m in required):
        raise ValueError('Need all three solo and six pair measurements')
    f = np.array(sorted(set.intersection(*(set(modes[m]) for m in required))))
    if len(f) < 30:
        raise ValueError('Too few common frequency points')
    amps = {float(modes[m][hz]['amp']) for m in required for hz in f}
    if len(amps) != 1 or min(amps) <= 0:
        raise ValueError('Sweep must use one positive drive amplitude')
    amp = amps.pop()
    get = lambda m,k: np.array([float(modes[m][hz][k]) for hz in f])
    S = np.array([get(c,'h1') / amp for c in CHANNELS]).T
    G = np.zeros((len(f),3,3))
    D = np.zeros_like(S)
    complete = f * 3 <= 747
    for i,c in enumerate(CHANNELS):
        G[:,i,i] = S[:,i]**2
        h2,h3 = get(c,'h2'),get(c,'h3')
        D[:,i] = np.hypot(np.maximum(h2,0),np.maximum(h3,0)) / np.maximum(get(c,'h1'),1e-6)
        # Unknown harmonic bins are not evidence of a clean response.
        D[~complete,i] = np.maximum(D[~complete,i], .5)
    consistency=[]
    for i,j in [(0,1),(0,2),(1,2)]:
        a,b=CHANNELS[i],CHANNELS[j]
        plus,minus = get(f'{a}+{b}','h1')/amp,get(f'{a}-{b}','h1')/amp
        G[:,i,j]=G[:,j,i]=(plus**2-minus**2)/4
        consistency.append(np.abs(plus**2+minus**2-2*(S[:,i]**2+S[:,j]**2)) /
                           np.maximum(2*(S[:,i]**2+S[:,j]**2),1))
    # Nonlinear/inconsistent pairs can give impossible negative energy. Record
    # their error; project correlation onto PSD without changing solo gains.
    corrections=[]
    for k in range(len(f)):
        C=G[k]/np.maximum(np.outer(S[k],S[k]),1e-9)
        v,U=np.linalg.eigh(C)
        P=(U*np.maximum(v,1e-6))@U.T
        P/=np.sqrt(np.outer(np.diag(P),np.diag(P)))
        corrections.append(float(np.linalg.norm(P-C)))
        G[k]=P*np.outer(S[k],S[k])
    return f,S,G,D,amp,np.array(consistency),corrections

def response(taps,f):
    # Real signed response after removal of common delay; do not take abs per path.
    return np.array([np.real(freqz(h,worN=f,fs=FS)[1]*np.exp(2j*np.pi*f/FS*((TAPS-1)/2))) for h in taps]).T

def design(path,out,lo=50,hi=240,drive_cap=None,low_reference=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    f,S,G,D,amp,consistency,corrections=load_model(path)
    cap=amp if drive_cap is None else drive_cap
    if not 0 < cap <= amp:
        raise ValueError('Do not extrapolate above measured amplitude')
    band=(f>=lo)&(f<=hi)
    # LFi is an asymmetric impact actuator. Never include it in the bipolar
    # sustained-response optimizer; its positive pulse is handled separately.
    G[:,0,:]=0;G[:,:,0]=0
    weights=[]
    for k in range(len(f)):
        # Generalized eigenvector: acceleration per distortion-weighted drive.
        penalty=np.diag(1+(D[k]/.2)**2)
        _,vec=eigh(G[k,1:,1:],penalty[1:,1:])
        w=np.r_[0.,vec[:,-1]]
        if weights and np.dot(w,weights[-1])<0: w=-w
        if not weights and w[np.argmax(abs(w))]<0: w=-w
        weights.append(w/np.max(abs(w)))
    w=gaussian_filter1d(np.array(weights),1.1,axis=0)
    raw=np.sqrt(np.maximum(np.einsum('fi,fij,fj->f',w,G,w),1e-9))
    target=float(np.min(raw[band]*cap/np.max(abs(w[band]),axis=1))*.75)
    desired=w*target/raw[:,None]
    taper=np.clip((f-(lo*.5))/(lo*.5),0,1)*np.clip((min(500,hi*1.5)-f)/(min(500,hi*1.5)-hi),0,1)
    desired*=taper[:,None]
    predict=lambda H: np.sqrt(np.maximum(np.einsum('fi,fij,fj->f',H,G,H),1e-12))
    if low_reference:
        fl,Sl,_,_,al,*_=load_model(low_reference)
        Sl=np.array([np.interp(f,fl,Sl[:,i]) for i in range(3)]).T
        exponent=np.clip(np.log(np.maximum(S*amp,1e-6)/np.maximum(Sl*al,1e-6))/np.log(amp/al),.7,3.)
        corr=np.clip(G[:,1,2]/np.maximum(S[:,1]*S[:,2],1e-9),-1,1)
        def predict(H):
            mag=S*amp*(abs(H)/amp)**exponent
            return np.sqrt(np.maximum(mag[:,1]**2+mag[:,2]**2+
                2*corr*mag[:,1]*mag[:,2]*np.sign(H[:,1]*H[:,2]),1e-12))
        # LF remains positive; HF negative throughout. No polarity reversal
        # through a resonance. Nonlinear amplitude fit avoids assuming that
        # a sub-threshold HF command reproduces a scaled high-drive response.
        target=min(150.,float(np.min(S[band,2]*cap))*.85)
        def unpack(x): return np.column_stack((np.zeros(len(f)),-x[:len(f)],x[len(f):]))
        def residual(x):
            H=unpack(x);out_=predict(H)
            return np.r_[(out_-target*taper)/max(target,1),
                5*(H[f<145,1]/cap),
                8*np.maximum(.11-H[band,2],0)/cap,
                .10*(np.diff(H[:,1:],axis=0)/cap).ravel(),
                .02*(H[:,1:]/cap*(1+D[:,1:])).ravel()]
        sol=least_squares(residual,np.full(len(f)*2,cap*.3),bounds=(0,cap),max_nfev=300)
        desired=unpack(sol.x)
    grid=np.r_[0,f,1000]
    for iteration in range(5):
        taps=np.array([firwin2(TAPS,grid,np.r_[0,desired[:,i],0],fs=FS) for i in range(3)])
        # Exact DC rejection, preserving symmetry.
        taps[:,TAPS//2]-=taps.sum(axis=1)
        H=response(taps,f)
        pred=predict(H)
        correction=np.ones(len(f));correction[band]=np.clip(target/pred[band],.7,1.4)
        desired*=gaussian_filter1d(correction,.7)[:,None]
    if low_reference:
        half=TAPS//2
        def unpack_taps(x):
            sides=x.reshape(2,half)
            return np.vstack((np.zeros(TAPS),np.column_stack((sides,-2*sides.sum(axis=1),sides[:,::-1]))))
        # Fit the realizable FIR itself, rather than only a frequency table.
        dense=np.linspace(0,1000,257)
        stop=(dense<lo*.35)|(dense>min(500,hi*1.7))
        def fit_residual(x):
            t=unpack_taps(x);H=response(t,f);Hd=response(t,dense)
            return np.r_[((predict(H)-target*taper)/max(target,1)),
                5*(H[f<145,1]/cap),
                8*np.maximum(.11-H[band,2],0)/cap,
                8*np.maximum(abs(Hd[:,1:])-cap,0).ravel()/cap,
                .2*(Hd[stop,1:]/cap).ravel(),
                .1*(np.maximum(H[:,1],0)/cap),.1*(np.minimum(H[:,2],0)/cap),
                .03*((H[:,1:]-desired[:,1:])/cap).ravel()]
        fit=least_squares(fit_residual,taps[1:,:half].ravel(),max_nfev=200)
        taps=unpack_taps(fit.x)
    # Use available sine headroom. Arbitrary multitone/impulse overload is
    # handled by the linked peak limiter, and is NOT claimed linear/flat.
    scale=min(1,cap/np.max(abs(response(taps,np.linspace(0,1000,4097)))))
    taps*=scale
    q=np.rint(taps*(1<<28)).astype(np.int64)
    q[:,TAPS//2]-=q.sum(axis=1)
    taps=q/(1<<28)
    H=response(taps,f)
    pred=predict(H)
    db=20*np.log10(pred[band]/np.exp(np.mean(np.log(pred[band]))))
    stats=dict(source=str(Path(path).resolve()),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        sample_rate=FS,taps=TAPS,delay_ms=1000*(TAPS-1)/2/FS,band_hz=[lo,hi],
        calibration_amp=amp,peak_bound=cap,channels=list(NAMES),slots=[0,2,3],
        low_reference=low_reference,
        predicted_geomean_mg=float(np.exp(np.mean(np.log(pred[band])))),
        predicted_ripple_db=float(np.ptp(db)),max_l1=float(np.max(np.sum(abs(taps),axis=1))),
        pair_energy_error_median=float(np.median(consistency[:,band])),
        pair_energy_error_max=float(np.max(consistency[:,band])),
        max_psd_correction=max(corrections),common_points=len(f),
        lfi_role='Positive transient pulse only; zero sustained FIR coefficients',
        status='PROVISIONAL: nonlinear amplitude dependence; measured closed-loop validation required',
        limitations=['Three-axis acceleration magnitude, not one-axis force or acoustic flatness.',
        'No absolute phase available; same-delay symmetric FIR paths only.',
        'H2/H3 are partial distortion indicators, not total THD.',
        'No clean-output extrapolation to full scale. No response claim outside measured band.'])
    (out/'design.json').write_text(json.dumps(stats,indent=2))
    np.savez(out/'filter.npz',taps=taps,f=f,predicted_mg=pred,drive=H)
    header='#pragma once\n#include <stdint.h>\nnamespace ThreewayCoefficients {\n'
    header+=f'constexpr int kRate={FS}, kTaps={TAPS};\nconstexpr int kSlots[3]={{0,2,3}};\n'
    header+=f'constexpr int32_t kPeakQ15={int(cap*32767)};\nconstexpr int32_t kFir[3][kTaps]={{\n'
    header+=',\n'.join('{'+','.join(map(str,row))+'}' for row in q)+'\n};\n}\n'
    (out/'threeway_coefficients.h').write_text(header)
    fig,axes=plt.subplots(3,1,figsize=(10,10),sharex=True)
    for i,name in enumerate(NAMES):
        axes[0].semilogx(f,S[:,i]*amp,label=name)
        axes[1].semilogx(f,H[:,i],label=name)
    axes[0].set_ylabel('Measured fundamental (mg)');axes[0].legend()
    axes[1].set_ylabel('Filter drive / unit input');axes[1].legend()
    axes[2].semilogx(f,pred,label='Predicted combined (not validated)')
    axes[2].set_ylabel('Acceleration (mg)');axes[2].set_xlabel('Frequency (Hz)');axes[2].legend()
    for ax in axes:
        ax.axvspan(lo,hi,alpha=.08,color='green');ax.grid(True,alpha=.3)
    fig.suptitle(f'Three-actuator candidate: {lo}-{hi} Hz, {stats["predicted_ripple_db"]:.1f} dB predicted ripple')
    fig.tight_layout();fig.savefig(out/'response.png',dpi=160);plt.close(fig)
    print(json.dumps(stats,indent=2))
    return stats

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv');ap.add_argument('--out',required=True)
    ap.add_argument('--band',nargs=2,type=float,default=[50,240]);ap.add_argument('--cap',type=float)
    ap.add_argument('--low-reference')
    a=ap.parse_args();design(a.csv,a.out,*a.band,a.cap,a.low_reference)
