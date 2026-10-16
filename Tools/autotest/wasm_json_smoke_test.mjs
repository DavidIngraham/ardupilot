// AI-assisted regression test for the JSON:wasm lockstep mailbox.
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';

const {default:create} = await import(pathToFileURL(process.argv[2]));
const module = await create({
    arguments: ['--model', 'JSON:wasm', '--rate', '200', '--serial0', 'wasm', '--serial1', 'none', '--serial2', 'none'],
    print: () => {}, printErr: console.error,
});
const malloc = module.cwrap('ardupilot_malloc', 'number', ['number']);
const read = module.cwrap('ardupilot_json_read', 'number', ['number', 'number']);
const write = module.cwrap('ardupilot_json_write', 'number', ['number', 'number', 'number']);
const buffer = malloc(8192);
const deadline = Date.now() + 15000;
let previous = null;
for (let step = 0; step < 10; step++) {
    let length = 0;
    while (!length && Date.now() < deadline) {
        assert.equal(read(buffer, 1), 0, 'Short reads must not consume a frame');
        length = read(buffer, 72);
        if (!length) await new Promise(resolve => setTimeout(resolve, 1));
    }
    assert.ok(length === 40 || length === 72, 'Must receive a complete servo packet');
    const packet = new DataView(module.HEAPU8.buffer, buffer, length);
    const frame = packet.getUint32(4, true);
    if (previous !== null) assert.equal(frame, previous + 1);
    assert.equal(read(buffer, 72), 0, 'A frame is handed to the host only once');
    const sensors = new TextEncoder().encode('\n' + JSON.stringify({
        timestamp: (step + 1) / 200,
        imu: {gyro: [0, 0, 0], accel_body: [0, 0, -9.80665]},
        position: [0, 0, -100], attitude: [0, 0, 0], velocity: [0, 0, 0],
    }) + '\n');
    module.HEAPU8.set(sensors, buffer);
    assert.equal(write(frame + 1, buffer, sensors.length), 0, 'Reject wrong frame');
    assert.equal(write(frame, buffer, 8193), 0, 'Reject oversized sensor data');
    assert.equal(write(frame, buffer, 2), 0, 'Reject incomplete sensor data');
    assert.equal(write(frame, buffer, sensors.length), sensors.length);
    assert.equal(write(frame, buffer, sensors.length), 0, 'Reject duplicate sensor response');
    previous = frame;
}
console.log('PASS: ten lockstep frames, bounded buffers, stale and duplicate rejection');
process.exit(0);
