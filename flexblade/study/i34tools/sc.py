import sys,json
sys.path.insert(0,'flexblade/study')
import control_quality as cq
from flexblade import robust
from flexblade.schema import Design
cq.WORKERS=3
def go(path,js):
    d=Design.model_validate_json(open(path).read())
    cs=cq.cal_eval(d,js,"continuous",5)
    s=cq.cal_summary(cs); c=s["control_quality_cal"]
    print(path.split('/')[-1],"cr_pl",s["margin"]["cr_pl"],"feas",c["feasible_pl"],"viol",c["viol_hard"],"ph",c["phase_res_deg"],"gain",c["gain_res"],"auth",c["authority_min"],"mech",c["mech_margin_min"],"flip",c["mech_side_flip"],"ratio",c["cr_pl_ratio_min"],flush=True)
if __name__=="__main__":
    go(sys.argv[1], cq.J10 if sys.argv[2]=="10" else cq.JS)
