"""夹爪接触观测：每指接触力与双指平均夹持载荷，单位N。"""
import math


def gripping_force(left_finger_n, right_finger_n):
    values = [float(left_finger_n), float(right_finger_n)]
    if any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError('finger contact forces must be finite and nonnegative')
    return {'finger_contact_n': values, 'grip_force_n': sum(values)/2,
            'bilateral_contact': all(v > 1e-5 for v in values),
            'source': 'simulated_finger_contact', 'definition': 'mean_of_two_finger_contact_magnitudes'}


def create_finger_sensors(dt=1/240):
    from isaacsim.sensors.physics import ContactSensor
    from pxr import Usd, UsdPhysics, PhysxSchema
    import omni.usd
    stage=omni.usd.get_context().get_stage()
    sensors={}
    for side in ('left','right'):
        for i in (1,2):
            path=f'/micromanipulator/link_finger{i}_{side}'
            prim=stage.GetPrimAtPath(path)
            if not prim: raise ValueError(f'missing finger body {path}')
            # Sensors observe existing collision geometry. Never add a parent hull:
            # that would fill finger recesses and duplicate dedicated child shapes.
            if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(prim)):
                raise ValueError(f'finger has no collision shape {path}')
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            # A fresh runtime sensor uses the body's contact reports directly;
            # avoid the legacy existing-prim wrapper's parent CollisionAPI check.
            if stage.GetPrimAtPath(path+'/GripContact'):stage.GetPrimAtPath(path+'/GripContact').SetActive(False)
            sensors[(side,i)]=ContactSensor(prim_path=path+'/GripContactRuntime',dt=dt,
                                            min_threshold=0.,max_threshold=1000.,radius=-1.)
    return sensors


def read_grip_sensors(sensors):
    result={}
    for side in ('left','right'):
        forces = []
        for i in (1, 2):
            sensor = sensors[(side, i)]
            reading = sensor._contact_sensor_interface.get_sensor_reading(sensor.prim_path, use_latest_data=True)
            if not reading.is_valid:
                raise ValueError(f'invalid current contact reading for {sensor.prim_path}')
            forces.append(reading.value)
        result[side] = gripping_force(*forces)
    return result
