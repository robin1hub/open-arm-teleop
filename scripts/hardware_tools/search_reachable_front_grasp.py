#!/usr/bin/env python3
"""Search reachable right-arm forward grasp targets from current real state."""

import itertools
import pathlib
import time

import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams, Kinematics
from openarm_driver import Config, SingleArmDriver


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]


def make_kin():
    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(), mode="right",
        frame_right="right_ee_control_point", frame_type_right="site",
        frame_left="left_ee_control_point", frame_type_left="site", keyframe="home",
    )
    return Kinematics(setup, IKParams(max_iters=10, dt=0.1, damping=0.1,
                                      posture_cost=0.0, orientation_cost=0.1,
                                      lm_damping=0.01))


def solve_candidate(qr, ql, offset):
    kin = make_kin()
    pr0 = kin.fk("right", qr)
    target = pr0.copy(); target[:3] += offset
    kin.sync(np.concatenate([qr, ql]).astype(np.float32))
    previous = np.concatenate([qr, ql]); max_step = 0.0; result = None
    for alpha in np.linspace(0.0, 1.0, 151)[1:]:
        pr = pr0 + alpha * (target - pr0)
        kin.set_target("right", pr)
        result = kin.solve()
        if result is None or not np.all(np.isfinite(result)): return None
        max_step=max(max_step,float(np.max(np.abs(result-previous)))); previous=result
    actual_r=kin.fk("right",result[:8])
    return pr0,target,result,max_step,float(np.linalg.norm(actual_r[:3]-target[:3]))


def main():
    cfg=Config(PROJECT_ROOT / "config/openarm_safe_current.yaml")
    states={}
    for side in ("right","left"):
        d=SingleArmDriver(side+"_arm",cfg); samples=[]
        for _ in range(8): samples.append(d.fetch_position(refresh=True)); time.sleep(.02)
        states[side]=np.asarray(samples)[-1]; d.openarm.disable_all()
    limits={s:np.asarray(cfg.get_joint_limits(s+"_arm")) for s in states}
    candidates=[]
    for dx,dz in itertools.product((0.02,0.03,0.05,0.08,0.10,0.12),(-0.10,-0.08,-0.05,-0.02,0.0,0.03,0.05)):
        out=solve_candidate(states["right"],states["left"],np.array([dx,0.0,dz]))
        if out is None: continue
        p0,target,result,max_step,error=out; qr=result[:8]; ql=result[8:]
        within=(np.all((qr>=limits["right"][:,0])&(qr<=limits["right"][:,1])) and
                np.all((ql>=limits["left"][:,0])&(ql<=limits["left"][:,1])))
        travel=np.abs(qr-states["right"])
        if within and max_step<=0.03 and error<=0.010 and np.max(travel[:7])<=0.60:
            score=dx-0.3*np.max(travel[:7])-2.0*error
            candidates.append((score,dx,dz,p0,target,qr,travel,max_step,error))
    if not candidates: raise RuntimeError("no candidate passed constraints")
    candidates.sort(key=lambda x:x[0],reverse=True)
    for rank,c in enumerate(candidates[:5],1):
        _,dx,dz,p0,target,qr,travel,max_step,error=c
        print(f"#{rank}: dx={dx:.3f} m dz={dz:.3f} m error={error*1000:.2f} mm maxstep={max_step:.5f}")
        print("  start pose :",np.array2string(p0,precision=5))
        print("  target pose:",np.array2string(target,precision=5))
        print("  joints deg :",np.array2string(np.degrees(qr),precision=2))
        print("  delta deg  :",np.array2string(np.degrees(qr-states['right']),precision=2))
    print("SUCCESS: reachable front-grasp candidates found.")


if __name__=="__main__": main()
