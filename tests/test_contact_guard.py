import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaacsim_tools'))
from micromani_contact_guard import ContactGuard

def sample(i, force=(.01, .01)):
    return dict(stale=False, sequence=i, simulation_time=i/30,
                grip={'left': {'finger_contact_n': list(force)},
                      'right': {'finger_contact_n': [0., 0.]}},
                force_torque={'left_wrist': {'saturated': False},
                              'right_wrist': {'saturated': False}})

def test_confirmation_requires_distinct_frames_and_resets_on_loss():
    g=ContactGuard('left')
    assert not g.evaluate(sample(1),0)['bilateral_confirmed']
    for _ in range(10):
        assert not g.evaluate(sample(1),0)['bilateral_confirmed']
    for i in range(2,5):
        assert not g.evaluate(sample(i),0)['bilateral_confirmed']
    assert g.evaluate(sample(5),0)['bilateral_confirmed']
    assert not g.evaluate(sample(6,(.01,0)),0)['bilateral_confirmed']
    assert not g.evaluate(sample(7),0)['bilateral_confirmed']

def test_limits_staleness_and_bad_numbers_stop():
    for forces, wrist, reason in [((.06,.01),0,'finger_force_limit'),
                                  ((.025,0),0,'unilateral_force_limit'),
                                  ((.01,.01),.11,'wrist_residual_limit'),
                                  ((float('nan'),.01),0,'invalid_sensor_value')]:
        assert ContactGuard('left').evaluate(sample(1,forces),wrist)['reason']==reason
    s=sample(1);s['stale']=True
    assert ContactGuard('left').evaluate(s,0)['stop']

def test_unexpected_other_contact_and_clock_regression_stop():
    s=sample(1);s['grip']['right']['finger_contact_n']=[.002,0]
    assert ContactGuard('left').evaluate(s,0)['stop']
    g=ContactGuard('left');g.evaluate(sample(10),0)
    assert g.evaluate(sample(9),0)['reason']=='sensor_clock_regressed'


def test_fault_latches_until_new_guard_is_explicitly_created():
    g=ContactGuard('left')
    assert g.evaluate(sample(1,(.06,.01)),0)['stop']
    assert g.evaluate(sample(2),0)['stop']
    s=sample(1);s['sequence']='bad'
    assert ContactGuard('left').evaluate(s,0)['reason']=='invalid_sequence'

def test_held_other_gripper_requires_fresh_loss_samples():
    from micromani_contact_guard import ContactGuard
    state = {'stale': False, 'sequence': 1, 'simulation_time': 1.,
             'grip': {'left': {'finger_contact_n': [.008, .008]},
                      'right': {'finger_contact_n': [0., 0.]}},
             'force_torque': {'left_wrist': {'saturated': False},
                              'right_wrist': {'saturated': False}}}
    guard = ContactGuard('right', holding_other=True)
    assert not guard.evaluate(state, 0.)['stop']
    state['grip']['left']['finger_contact_n'] = [0., .008]
    state.update(sequence=2, simulation_time=2.)
    assert not guard.evaluate(state, 0.)['stop']
    for _ in range(10):
        assert not guard.evaluate(state, 0.)['stop']
    state.update(sequence=3, simulation_time=3.)
    assert not guard.evaluate(state, 0.)['stop']
    state.update(sequence=4, simulation_time=4.)
    assert guard.evaluate(state, 0.)['reason'] == 'lost_other_gripper_contact'
    state['grip']['left']['finger_contact_n'] = [.008, .008]
    assert guard.evaluate(state, 0.)['stop']
