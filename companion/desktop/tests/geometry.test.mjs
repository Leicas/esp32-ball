import assert from 'node:assert/strict';
import test from 'node:test';
import { tubeScreenAngle, tubeSideViewEndpoints, rotateBodyToWorld, projectWorld } from '../src/geometry.ts';

test('positive tube gravity lowers the right end in Canvas coordinates', () => {
  assert.ok(Math.sin(tubeScreenAngle(.5)) > 0);
  assert.ok(Math.sin(tubeScreenAngle(-.5)) < 0);
  assert.equal(tubeScreenAngle(0), 0);
});

test('side-view tube keeps the same screen-space length at every incline', () => {
  const length = 300;
  for (const sine of [-1, -.9, -.5, 0, .5, .9, 1]) {
    const [[x0, y0], [x1, y1]] = tubeSideViewEndpoints(400, 200, length, sine);
    assert.ok(Math.abs(Math.hypot(x1 - x0, y1 - y0) - length) < 1e-9);
    if (sine > 0) assert.ok(y1 > y0);
    if (sine < 0) assert.ok(y1 < y0);
  }
});

test('marble gravity points down after rotating from body to world', () => {
  for (const raw of [[1,0,0,0], [1,1,0,0], [.8,.1,-.4,.3], [.2,-.8,.5,.1]]) {
    const n = Math.hypot(...raw);
    const [w,x,y,z] = raw.map(v => v/n);
    const gravity = [2*(x*z-w*y), 2*(w*x+y*z), w*w-x*x-y*y+z*z];
    const world = rotateBodyToWorld(gravity, raw);
    assert.ok(Math.abs(world[0]) < 1e-12);
    assert.ok(Math.abs(world[1]) < 1e-12);
    assert.ok(Math.abs(world[2]-1) < 1e-12);
    const projected = projectWorld(world);
    assert.ok(projected[1] > .99);
  }
});

test('box floor is below its ceiling and invalid quaternion is stable', () => {
  assert.ok(projectWorld([0,0,25])[1] > projectWorld([0,0,0])[1]);
  assert.deepEqual(rotateBodyToWorld([1,2,3],[0,0,0,0]), [1,2,3]);
});
