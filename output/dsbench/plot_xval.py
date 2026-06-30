#!/usr/bin/env python3
"""Cross-validation overlay: 0.5m vs full dense map, both reject-off, full route at rate 1.0."""
import csv,numpy as np,matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
R="/ws/output/dsbench/runs"; PLY="/ws/gt_map/gt_map.ply"
OUT="/ws/output/dsbench/traj_xval_norej.png"
def lp(p,s=20):
    raw=open(p,"rb").read(); e=raw.index(b"end_header\n")+len(b"end_header\n")
    return np.frombuffer(raw[e:],np.float32).reshape(-1,4)[::s,:3]
def lt(tag):
    t=[];x=[]
    for r in csv.reader(open(f"{R}/{tag}/pose.csv")):
        if r[0]=="stamp": continue
        ts=float(r[0])
        if ts<1e6: continue
        t.append(ts);x.append([float(r[1]),float(r[2]),float(r[3])])
    t=np.array(t);x=np.array(x);o=np.argsort(t);return t[o],x[o]
m=lp(PLY); qt,qx=lt("us050_norej"); ft,fx=lt("full_norej")
fig,ax=plt.subplots(1,2,figsize=(20,9))
ax[0].scatter(m[:,0],m[:,1],c="0.85",s=0.3,linewidths=0)
ax[0].plot(qx[:,0],qx[:,1],"-",color="#d62728",lw=1.6,label=f"0.5m map ({len(qx)} poses)")
ax[0].plot(fx[:,0],fx[:,1],"--",color="#1f77b4",lw=1.3,label=f"full map ({len(fx)} poses)")
ax[0].scatter(*qx[0,:2],c="lime",s=150,edgecolors="k",zorder=5,label="start")
ax[0].scatter(*qx[-1,:2],c="magenta",s=150,marker="s",edgecolors="k",zorder=5,label="end")
ax[0].set_title("Full route @ rate 1.0, reject-off: 0.5m vs full map (top-down XY)")
ax[0].set_xlabel("x [m]");ax[0].set_ylabel("y [m]");ax[0].axis("equal");ax[0].legend(loc="upper right")
# z vs time for both
ax[1].plot(qt-qt[0],qx[:,2],"-",color="#d62728",lw=1.2,label="0.5m map")
ax[1].plot(ft-ft[0],fx[:,2],"--",color="#1f77b4",lw=1.2,label="full map")
ax[1].set_title("Height vs time — both maps agree through the elevated 320-425s zone")
ax[1].set_xlabel("bag time [s]");ax[1].set_ylabel("z [m]");ax[1].grid(alpha=0.3);ax[1].legend()
plt.tight_layout();plt.savefig(OUT,dpi=100);print("saved",OUT)
