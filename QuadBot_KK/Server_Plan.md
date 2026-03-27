Sure! Here's a summary of what we covered:

## The Situation
Team is behind schedule, simulation phase was skipped due to a teammate leaving, and the plan is to go straight to hardware assembly. Assembly is expected to take 7-10 days, and you're using that time to plan ahead.

## IK & Coordinates
Instead of thinking in world coordinates, work in **coordinates relative to each hip joint**. Define a neutral foot position based on your physical link lengths, and express all motion as offsets from that neutral. This makes the math simple and intuitive without needing simulation.

## Single Leg Controller
Plan is to build a real-time tuning system on the ESP32 so you can adjust parameters (like neutral position) on the fly without reflashing. Architecture is:
- ESP32 hosts a WiFi access point
- Browser-based UI with sliders
- Parameters sent to ESP32 in real time, which recomputes IK and moves servos immediately

## WebSockets
Chosen over plain HTTP for the controller because it's a persistent two-way connection — connect once, communicate freely. Low latency, good for real-time control, and will be useful later for streaming IMU data and gait visualization.

## Other Communication Protocols
- **HTTP** — simple, one-shot, good for basic commands
- **MQTT** — pub/sub, great for multi-device IoT networks
- **UDP** — fastest, no guarantees, used in high-frequency streaming
- **Serial** — simplest, tethered, best for early debugging
- **ROS** — industry standard for research robotics

## WiFi Reliability
Key risks are interference, ESP32 getting too busy to handle WiFi, and power issues from servos. Main fixes are:
- **Run WiFi on core 0, robot code on core 1** (most important)
- **Separate power supply for servos**
- Build auto-reconnect into the client
- Design for fail-safe behavior on disconnect

---

Next natural step would be planning the actual single leg control loop — the software flow from receiving a parameter update to moving the servo. Want to continue there?