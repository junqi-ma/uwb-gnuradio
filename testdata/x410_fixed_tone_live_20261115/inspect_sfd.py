import numpy as np
from scipy.signal import correlate,find_peaks
from pathlib import Path
p=Path(__file__).parent;x=np.fromfile(p/'first_window.cf32',np.complex64);t=np.fromfile('testdata/reference_preamble_code10_998p4.cf32',np.complex64)
c=correlate(x,t,mode='valid',method='fft');e=np.r_[0,np.cumsum(abs(x.astype(np.complex128))**2)];q=abs(c)**2/np.maximum((e[len(t):]-e[:-len(t)])*sum(abs(t)**2),1e-30)
# Inspect one symbol grid anchored on first preamble peak.
a=int(np.flatnonzero(q[18000:30000]>.35)[0]+18000)
print('anchor',a,'metric',q[a]); rows=[]
for i in range(-5,290):
 pred=a+1016*i
 if pred<20 or pred+20>=len(c):continue
 j=pred-12+np.argmax(q[pred-12:pred+13]);rows.append((i,j,q[j],float(np.angle(c[j])),abs(c[j])))
for r in rows:
 if r[0]<5 or r[0]>245:print(r)
np.savez(p/'sfd_grid.npz',rows=np.array(rows),q=q,c=c)
