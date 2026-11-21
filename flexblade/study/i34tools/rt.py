import sys,json
from flexblade import rotor
from flexblade.schema import Design
import sc, control_quality as cq
src,out,lf,wf,sk,js=sys.argv[1:7]
d=Design.model_validate_json(open(src).read())
fx=d.flexure
upd=dict(length=fx.length*float(lf),width=fx.width*float(wf),skew_deg=fx.skew_deg+float(sk))
d=d.model_copy(update=dict(flexure=fx.model_copy(update=upd)))
info={}
d=rotor.resolve_strip_preset(d,iters=14,info=info)
print("resolve resid",[round(x,3) for x in info["resid_deg"]][-3:],flush=True)
open(out,'w').write(d.model_dump_json(indent=1))
sc.go(out,cq.J10 if js=="10" else cq.JS)
