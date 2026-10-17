# X Chassis Web Console

This is a static Web Serial upper computer for the C-board + CM4M X-layout chassis.

It uses the same versioned `A5 5A` protocol as the ROS 2 bridge and lower
controller. Manual velocity commands carry the streaming flag and are refreshed
every 100 ms; the C board clears them after 300 ms without a fresh frame. The
browser retries a lost ACK with the same sequence number, so the lower
controller replays the response without executing a motion command twice.
During a route, a short `GET_POSE` loss is logged and retried without aborting
the waypoint; only 8 seconds without a valid pose ends the route. Any route
error now sends `CANCEL` and a zero velocity command before returning to idle.
Path execution uses a 35 cm look-ahead while the chassis is still moving, so
intermediate points do not force a full stop. Manual velocity frames are sent
from a latest-value stream and do not wait for each ACK before sending the next
frame. If the pose leaves the route by more than 28 cm or the lower controller
aborts a goal, the page projects the pose onto the remaining route and sends a
bounded recovery target ahead of that projection. Only repeated recovery
failures cause a cancel and zero-speed stop.
The `连续航点` switch sets the firmware blend flag on intermediate goals;
the final goal always keeps normal approach braking and settling.
It speaks the existing C-board protocol at 115200 8N1 and does not require Python or a local backend.

Use desktop Chrome or Edge over `https://` (GitHub Pages) or `http://localhost`.
Connect the computer-side wireless serial adapter, click `连接串口`, and grant the browser port permission.
The page automatically sends `CLEAR_ESTOP` after a successful connection. The only emergency control is the red `急停` button.

The custom path editor accepts map clicks in local coordinates `x=0..8 m`, `y=-2.5..2.5 m`.
Edit each waypoint's yaw and speed in the table, save/load JSON, then execute the path. The browser sends 25-byte `SET_GOAL` waypoints with `speed_limit_mm_s`; older 21-byte goals remain compatible. The C board performs the pose loop and CM4M's stored channel limits are the final actuator cap.

## GitHub Pages

The workflow in `.github/workflows/pages.yml` publishes this directory on pushes to `main` or `master`.
In the repository settings, set Pages to `GitHub Actions` once. Web Serial requires HTTPS, so the published Pages URL is the intended deployment target.
