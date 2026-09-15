import os
import sys, io, contextlib, statistics as st
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
with contextlib.redirect_stdout(io.StringIO()):
    import cpu_window, mem_growth
O=os.environ.get('HMR_OUT', os.path.expanduser('~/jetbot-slam/hmr_localisation/output/jetson_test'))+'/'
G=os.environ.get('GLIM_OUT', os.path.expanduser('~/glim-output'))+'/'
ROWS=[
 ("hmr_loc","stride 4","bunker",  [O+f'cyc_hmr_bunk_s4_r{i}.cpu.csv' for i in(1,2,3)], O+'1cv2_bunk_s4.cpu.csv'),
 ("hmr_loc","stride 4","curtmini",[O+f'cyc_hmr_curt_s4_r{i}.cpu.csv' for i in(1,2,3)], O+'1cv2_curt_s4.cpu.csv'),
 ("hmr_loc","stride 8","bunker",  [O+f'cyc_hmr_bunk_s8_r{i}.cpu.csv' for i in(1,2,3)], O+'1cv2_bunk_s8.cpu.csv'),
 ("hmr_loc","stride 8","curtmini",[O+f'cyc_hmr_curt_s8_r{i}.cpu.csv' for i in(1,2,3)], O+'1cv2_curt_s8.cpu.csv'),
 ("glim","odom cpu","bunker",  [G+f'cyc_glim_bunk_o_combo_r{i}.cpu.csv' for i in(1,2,3)], G+'opt1c_bunk_combo.cpu.csv'),
 ("glim","odom cpu","curtmini",[G+f'cyc_glim_curt_o_combo_r{i}.cpu.csv' for i in(1,2,3)], G+'opt1c_curt_combo.cpu.csv'),
 ("glim","odom gpu","bunker",  [G+f'cyc_glim_bunk_o_combogpu_r{i}.cpu.csv' for i in(1,2,3)], G+'gap1c_bunk_gpucombo.cpu.csv'),
 ("glim","odom gpu","curtmini",[G+f'cyc_glim_curt_o_combogpu_r{i}.cpu.csv' for i in(1,2,3)], G+'gap1c_curt_gpucombo.cpu.csv'),
 ("glim","full+loop","bunker",  [G+f'cyc_glim_bunk_o_combofull_r{i}.cpu.csv' for i in(1,2,3)], G+'optf1c_bunk_full.cpu.csv'),
 ("glim","full+loop","curtmini",[G+f'cyc_glim_curt_o_combofull_r{i}.cpu.csv' for i in(1,2,3)], G+'optf1c_curt_full.cpu.csv'),
]
def score(p):
    with contextlib.redirect_stdout(io.StringIO()):
        w=cpu_window.window(p); m=mem_growth.report(p)
    return w,m
print("%-8s %-10s %-9s | %-22s | %-22s | %s"%("system","mode","bag","CYCLONE (n=3 mean)","FASTDDS SHM (n=1)","delta"))
print("%-8s %-10s %-9s | %6s %6s %6s %5s | %6s %6s %6s %5s | %6s %7s"%("","","","cores","RSS","tail","still","cores","RSS","tail","still","dcores","dRSS"))
print("-"*118)
out=[]
for sysn,mode,bag,cyc,old in ROWS:
    cs=[score(p) for p in cyc]
    cc=[w['cores'] for w,_ in cs]; cr=[m['peak'] for _,m in cs]
    ct=[m['tail'] for _,m in cs]; cst=sum(bool(m['still']) for _,m in cs)>=2
    ow,om=score(old)
    dc=st.mean(cc)-ow['cores']; dr=st.mean(cr)-om['peak']
    print("%-8s %-10s %-9s | %6.2f %6.0f %+6.1f %5s | %6.2f %6.0f %+6.1f %5s | %+6.2f %+7.0f"%(
        sysn,mode,bag,st.mean(cc),st.mean(cr),st.mean(ct),"YES" if cst else "no",
        ow['cores'],om['peak'],om['tail'],"YES" if om['still'] else "no",dc,dr))
    out.append((sysn,mode,bag,st.mean(cc),min(cc),max(cc),st.mean(cr),st.mean(ct),cst,ow['cores'],om['peak'],om['tail'],om['still'],dc,dr))
print()
print("dcores: mean %+0.3f  max|.| %0.2f   |   dRSS: mean %+0.0f MB  range %+0.0f..%+0.0f"%(
    st.mean(r[13] for r in out),max(abs(r[13]) for r in out),
    st.mean(r[14] for r in out),min(r[14] for r in out),max(r[14] for r in out)))
