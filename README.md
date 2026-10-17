# X Chassis Web Console

This is a static Web Serial upper computer for the C-board + CM4M X-layout chassis.
It speaks the existing C-board protocol at 115200 8N1 and does not require Python or a local backend.

Use desktop Chrome or Edge over `https://` (GitHub Pages) or `http://localhost`.
Connect the computer-side wireless serial adapter, click `连接串口`, and grant the browser port permission.
The page automatically sends `CLEAR_ESTOP` after a successful connection. The only emergency control is the red `急停` button.

The custom path editor accepts map clicks in local coordinates `x=0..8 m`, `y=-2.5..2.5 m`.
Edit each waypoint's yaw in the table, save/load JSON, then execute the path. The browser sends absolute `SET_GOAL` waypoints; the C board performs the pose loop.

## GitHub Pages

The workflow in `.github/workflows/pages.yml` publishes this directory on pushes to `main` or `master`.
In the repository settings, set Pages to `GitHub Actions` once. Web Serial requires HTTPS, so the published Pages URL is the intended deployment target.
