import { describe, it, expect } from 'vitest';
import { yawToQuat, makeOdometry } from '../net/ros-bridge.js';

describe('ros-bridge helpers', () => {
  it('yawToQuat returns unit quaternion at 0', () => {
    const q = yawToQuat(0);
    expect(q).toEqual({ x: 0, y: 0, z: 0, w: 1 });
  });

  it('yawToQuat(π) flips sign as expected in simplified convention', () => {
    const q = yawToQuat(Math.PI);
    expect(Math.abs(q.y)).toBeCloseTo(1, 6);
    expect(Math.abs(q.w)).toBeCloseTo(0, 6);
  });

  it('makeOdometry sets frame ids and nested fields', () => {
    const msg = makeOdometry({
      frame_id: 'map',
      child_frame_id: 'base_link',
      position: { x: 1, y: 2, z: 3 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
      linear: { x: 0.1, y: 0.2, z: 0.3 },
      angular: { x: 0, y: 0, z: 0.01 }
    });
    expect(msg.header.frame_id).toBe('map');
    expect(msg.child_frame_id).toBe('base_link');
    expect(msg.pose.pose.position.z).toBe(3);
    expect(typeof msg.header.stamp.secs).toBe('number');
    expect(typeof msg.header.stamp.nsecs).toBe('number');
  });
});
