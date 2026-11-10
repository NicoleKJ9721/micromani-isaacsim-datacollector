"""Bounded synchronized joint trajectories. No pose reads or USD dependencies."""
import math
from micromani_sim_contract import validate_action

class JointMotion:
    # SI units: metres / radians, per finger for gripper.
    velocity = [.002,.002,.002,.1,.1,.1,.0005]*2
    acceleration = [.01,.01,.01,.5,.5,.5,.002]*2
    tolerance = [5e-6,5e-6,5e-6,5e-4,5e-4,5e-4,5e-6]*2
    def __init__(self, initial):
        self.reference=validate_action(initial);self.target=list(self.reference)
        self.start=list(self.reference);self.elapsed=0.;self.duration=0.;self.status='idle'
    def submit(self, target):
        target=validate_action(target)
        if self.status in ('moving','settling'):raise RuntimeError('motion busy; wait for motion terminal status or stop')
        self.start=list(self.reference);self.target=target;self.elapsed=0.
        self.duration=max([.05]+[max(1.875*abs(b-a)/v,math.sqrt(5.774*abs(b-a)/acc))
             for a,b,v,acc in zip(self.start,target,self.velocity,self.acceleration)])
        self.status='moving'
    def step(self, dt, measured):
        if self.status in ('moving','settling'):
            self.elapsed+=dt;u=min(1.,self.elapsed/self.duration)
            blend=u*u*u*(10+u*(-15+6*u))
            self.reference=[a+(b-a)*blend for a,b in zip(self.start,self.target)]
            if u>=1:
                self.status='settling'
                if all(abs(x-y)<=tol for x,y,tol in zip(measured,self.target,self.tolerance)):self.status='reached'
                elif self.elapsed>self.duration+2.:self.status='contact_or_tracking_error'
        return list(self.reference)
    def stop(self, measured, preserve_grippers=False):
        from micromani_lerobot_schema import CONTROL_ORDER,JOINT_LIMITS
        hold=validate_action([max(JOINT_LIMITS[k][0],min(JOINT_LIMITS[k][1],v)) for k,v in zip(CONTROL_ORDER,measured)])
        if preserve_grippers:
            # Retain the already-applied drive reference, not the future target.
            # Replacing it with measured position would unload an established grip.
            for index in (6,13):hold[index]=self.reference[index]
        self.reference=validate_action(hold)
        self.target=list(self.reference);self.status='stopped'
    def snapshot(self,measured):
        return dict(status=self.status,elapsed_s=self.elapsed,duration_s=self.duration,
                    reference=self.reference,target_error=[b-a for a,b in zip(measured,self.target)],
                    velocity_limits=self.velocity,acceleration_limits=self.acceleration)

def drive_targets(robot, indices, action):
    """Physical drives only. Never reset position or velocity during execution."""
    import numpy as np
    from micromani_lerobot_schema import control_action_to_raw_targets
    raw=control_action_to_raw_targets(action)
    robot.set_joint_position_targets(np.array([[v for v in raw.values()]],dtype=np.float32),
                                     joint_indices=np.array([indices[k] for k in raw]))

    # Feed forward gravity from the robot's own model on every controlled axis.
    # Held task objects are not included or queried. Position drives remain active.
    controlled=np.array([indices[k] for k in raw])
    gravity=robot.get_generalized_gravity_forces(joint_indices=controlled)
    if gravity is None or not np.isfinite(gravity).all():
        raise RuntimeError('invalid robot gravity compensation')
    robot.set_joint_efforts(np.asarray(gravity,dtype=np.float32),joint_indices=controlled)
