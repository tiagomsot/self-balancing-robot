"""Cross-adapter checks for the one compiled C++ implementation."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"ros2_ws/src/balancing_robot_sim/scripts"))
from shared_core import Controller, NativeBatch, ImuEstimator
from motor_model import MotorDriverModel, MotorParameters


class SharedAdapterTests(unittest.TestCase):
    def setUp(self):
        config=yaml.safe_load((ROOT/"ros2_ws/src/balancing_robot_sim/config/balance.yaml").read_text())
        self.p=config["balance_controller"]["ros__parameters"]
        fields=MotorParameters.__dataclass_fields__
        self.motor=MotorParameters(**{k:v for k,v in config["motor_driver"]["ros__parameters"].items() if k in fields})
        m=self.motor
        self.parameters=np.array([.04,math.radians(5),math.radians(35),m.command_full_pwm_torque_nm,
            m.pwm_levels,m.command_deadband,m.battery_voltage_v,m.driver_drop_intercept_v,
            m.back_emf_v_s_rad,m.resistance_ohm+m.driver_resistance_ohm,m.torque_constant_nm_a,m.viscous_drag_nm_s_rad])
        self.gain=np.array([self.p[k] for k in ("kp","kd","position_gain","velocity_gain","total_torque_limit_nm","ki")])

    def test_batch_adapter_matches_single_controller_and_motor_physics(self):
        tilts=np.radians([5,-5,10,-10])
        poses=np.zeros((4,9));velocities=np.zeros((4,8))
        poses[:,3],poses[:,5]=np.cos(tilts/2),np.sin(tilts/2)
        poses[:,7:9]=np.array([[.4,.5],[-.3,-.2],[.1,.2],[-.2,.4]])
        velocities[:,4]=[.1,-.2,.15,-.1]
        velocities[:,6:8]=[[3,5],[-4,-8],[7,-6],[-5,2]]
        batch=NativeBatch(self.p,np.tile(self.gain,(4,1)))
        held=np.zeros((4,6));held[:,0]=-np.sin(tilts);held[:,2]=np.cos(tilts)
        readings=held.copy();readings[:,4]=velocities[:,4]
        batch.prime_imu(held)
        torques,diagnostic=batch.update(velocities,readings,self.parameters)
        expected=[]
        for v,rest,raw in zip(velocities,held,readings):
            estimator=ImuEstimator();estimator.update(rest)
            pitch,rate,tilt,yaw,valid=estimator.update(raw)
            self.assertTrue(valid)
            result=Controller(self.p).update(pitch,rate,0,0,yaw,tilt,0,0,False,0,0,.005,False)
            sample=MotorDriverModel(self.motor).step(result[3:5],v[6:8],.005)
            expected.append([sample.left.torque_nm,sample.right.torque_nm])
        np.testing.assert_allclose(torques,expected,rtol=1e-10,atol=1e-12)
        self.assertFalse(diagnostic[:,3].any())

    def test_batch_fault_remains_latched_after_pose_recovers(self):
        v=np.zeros((1,8));readings=np.array([[-math.sin(1),0,math.cos(1),0,0,0]])
        batch=NativeBatch(self.p,self.gain[None,:])
        batch.prime_imu(readings)
        torque,diagnostic=batch.update(v,readings,self.parameters)
        self.assertEqual(diagnostic[0,3],1)
        np.testing.assert_array_equal(torque,[[0,0]])
        readings[:]=[[0,0,1,0,0,0]]
        torque,diagnostic=batch.update(v,readings,self.parameters)
        self.assertEqual(diagnostic[0,3],1)
        np.testing.assert_array_equal(torque,[[0,0]])

    def test_adapter_rejects_incompatible_joint_layout(self):
        batch=NativeBatch(self.p,self.gain[None,:])
        with self.assertRaises(ValueError):
            batch.update(np.zeros((1,8)),np.zeros((1,5)),self.parameters)

    def test_unmeasured_wheel_fields_cannot_change_motor_requests(self):
        normal=Controller(self.p).update(.05,.1,0,0,0,.05,0,0,False,0,0,.005,False)
        unavailable=Controller(self.p).update(.05,.1,np.nan,np.nan,0,.05,0,np.nan,False,0,0,.005,False)
        np.testing.assert_array_equal(normal,unavailable)
        self.assertTrue(normal[5])

    def test_ideal_body_velocities_are_not_controller_measurements(self):
        readings=np.tile([-.05,0,1,0,.1,0],(2,1))
        velocities=np.zeros((2,8));velocities[1,:6]=[200,-90,100,4,5,-2]
        batch=NativeBatch(self.p,np.tile(self.gain,(2,1)))
        batch.prime_imu(readings)
        torques,diagnostic=batch.update(velocities,readings,self.parameters)
        np.testing.assert_array_equal(torques[0],torques[1])
        self.assertEqual(diagnostic[0,2],diagnostic[1,2])

    def test_hardware_mode_rejects_nonzero_wheel_gains(self):
        wrong=dict(self.p,position_gain=.1)
        with self.assertRaises(ValueError):Controller(wrong)
        gains=self.gain.copy();gains[2]=.1
        with self.assertRaises(ValueError):NativeBatch(self.p,gains[None,:])

    def test_invalid_raw_sensor_values_cannot_be_hidden_by_quantization(self):
        estimate=ImuEstimator().update([np.nan,0,1,0,0,0])
        self.assertFalse(estimate[-1])

    def test_pitch_integral_does_not_wind_up_against_saturated_motors(self):
        controller=Controller(dict(self.p,ki=1.0,kp=1.0))
        for _ in range(500):
            result=controller.update(.2,0,0,0,0,.2,0,0,False,0,0,.005,False)
            self.assertAlmostEqual(result[1],.04)
        released=controller.update(0,0,0,0,0,0,0,0,False,0,0,.005,False)
        self.assertAlmostEqual(released[0],0)

    def test_near_free_fall_is_not_misread_as_an_inverted_robot(self):
        estimator=ImuEstimator();estimator.update([0,0,1,0,0,0])
        for _ in range(50):
            pitch,rate,tilt,yaw,valid=estimator.update([-.001,0,-.001,0,0,0])
            self.assertTrue(valid)
            self.assertAlmostEqual(pitch,0)
            self.assertAlmostEqual(tilt,0)
