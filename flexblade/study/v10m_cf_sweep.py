"""v10m (t=0.20mm, L x1) CF-stiffness lever sweep. usage: v10m_cf_sweep.py [out.json]"""
import sys,os,json
os.environ.setdefault("FLEXBLADE_GROUND_RES","1")
here=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,here)
import run_study as rs
from flexblade import robust, api
from flexblade.schema import Design
D0=Design.model_validate_json(open(here+"/designs/10in_tt_strap_steelx_v10m.json").read())
def build(f=None,bal=None,pc=None):
    fu=dict(thickness=0.2e-3); b=D0.blade
    for k,v in (f or {}).items(): fu[k]=getattr(D0.flexure,k)*v if v<5 else v
    if bal is not None:
        m,c=bal; bm=[dict(mass=m*1e-3,r_frac=0.2,chord_offset=c,normal_offset=s*0.01) for s in(1,-1)] if m>0 else []
        b=b.model_copy(update=dict(balance_masses=[type(b.balance_masses[0]).model_validate(x) for x in bm]))
    if pc is not None: b=b.model_copy(update=dict(precone_deg=pc))
    return D0.model_copy(update=dict(blade=b,flexure=D0.flexure.model_copy(update=fu)))
def run(args):
    name,kw=args
    d=build(**kw)
    try: d=rs.resolve_candidate(d,6)
    except Exception as e: return dict(name=name,err=str(e))
    ms=[]
    for T,dv in robust.variants(d,robust.THRUST_FACTORS,None,[3.76e-6],None):
        r=api.evaluate(dv,["trim","modes","harmonic","checks"]).model_dump(mode="json")
        ms.append((T,rs.metrics(dv,r,criterion="margin")))
    c=robust.combine([m for _,m in ms],[t for t,_ in ms])
    gd=[(m.get("util") or {}).get("fatigue_stress") for _,m in ms]
    return dict(name=name,cr=c.get("cr_pl"),res=c.get("res_margin"),which=c.get("res_which"),
      mech=[m.get("pitch_per_rev") for _,m in ms],good=max(x for x in gd if x is not None) if any(x is not None for x in gd) else None,
      viol=c.get("viol_hard"),jit=[(m.get("util") or {}).get("harmonic_jitter") for _,m in ms],kw=kw)
if __name__=="__main__":
    P=[("base",{})]
    for v in(.75,.9,1.1,1.25): P.append((f"hinge_r x{v}",dict(f=dict(hinge_radius=v))))
    for v in(.75,.9,1.1,1.25): P.append((f"pin_d x{v}",dict(f=dict(strap_pin_diameter=v))))
    for m,c in((1,.01),(2,.01),(1,-.01),(2,-.01)): P.append((f"bal {m}g c{c}",dict(bal=(m,c))))
    for pc in(1.3,): P.append((f"precone {pc}",dict(pc=pc)))
    extra=json.loads(sys.argv[2]) if len(sys.argv)>2 else []
    P=[(n,k) for n,k in extra] if extra else P
    from multiprocessing import Pool
    with Pool(3) as p: R=p.map(run,P,chunksize=1)
    for r in R: print(json.dumps(r,default=str),flush=True)
    json.dump(R,open(sys.argv[1],"w"),default=str)
