import time
import random
import socket
import network
from machine import Pin, PWM, time_pulse_us

AP_SSID = "XeRobot_ESP32"
AP_PASS = "12345678"

STA_SSID = ""
STA_PASS = ""

MANUAL_TIMEOUT = 400

ENA, ENB = 13, 33
IN1, IN2 = 12, 14
IN3, IN4 = 27, 26
LINE_PINS = (32, 35, 34, 4, 2)
LINE_DETECTED = 1
TRIG_PIN, ECHO_PIN = 5, 18

PWM_FREQ = 5000
MIN_PWM = 90

SPEED_MANUAL = 200
SPEED_MANUAL_TURN = 220

SPEED_BASE = 155
SPEED_SLOW = 120
SPEED_TURN = 185
OBSTACLE_DIST = 20
SLOW_DIST = 50
CLEAR_DIST = 35
MAX_DIST = 200.0
PING_INTERVAL = 50
CONFIRM_COUNT = 2
T_BRAKE, T_BACK, T_PAUSE = 150, 350, 150
T_TURN1 = 1000
T_SCAN = T_TURN1 // 2
T_ESCAPE = 3 * T_SCAN
STUCK_WINDOW = 4000
STUCK_TRIES = 3

LF_WEIGHT = (-4, -2, 0, 2, 4)
LF_SPEED_MAX = 160
LF_SPEED_MIN = 115
LF_PIVOT_SPEED = 160
LF_KP = 22.0
LF_KI = 0.0
LF_KD = 0.25
LF_I_LIMIT = 100.0
LF_INTERVAL = 4
LF_GAP_HOLD = 60
LF_SWEEP_MS = 700
LF_LOST_TIMEOUT = 3500
LF_PIVOT_TIMEOUT = 1200

in1, in2 = Pin(IN1, Pin.OUT), Pin(IN2, Pin.OUT)
in3, in4 = Pin(IN3, Pin.OUT), Pin(IN4, Pin.OUT)
pwm_l = PWM(Pin(ENA), freq=PWM_FREQ, duty_u16=0)
pwm_r = PWM(Pin(ENB), freq=PWM_FREQ, duty_u16=0)

line_sensors = [Pin(p, Pin.IN) for p in LINE_PINS]
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _drive_side(pin_a, pin_b, pwm, speed):
    speed = clamp(int(speed), -255, 255)
    duty = abs(speed)
    if duty < MIN_PWM // 2:
        speed = 0
        duty = 0
    elif duty < MIN_PWM:
        duty = MIN_PWM
    pin_a.value(1 if speed > 0 else 0)
    pin_b.value(1 if speed < 0 else 0)
    pwm.duty_u16(duty * 257)


def drive(left, right):
    _drive_side(in1, in2, pwm_l, left)
    _drive_side(in3, in4, pwm_r, right)


def stop_robot():
    drive(0, 0)


def pivot(direction, speed):
    drive(direction * speed, -direction * speed)


def read_distance_cm():
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 12000)
    if duration < 0:
        return MAX_DIST
    return duration * 0.0343 / 2.0


def read_distance_median():
    v = []
    for _ in range(3):
        v.append(read_distance_cm())
        time.sleep_ms(12)
    v.sort()
    return v[1]


class ObstacleAvoider:
    (DRIVE, BRAKE, BACK, PAUSE, TURN_L, MEAS_L,
     TURN_R, MEAS_R, TURN_BEST, ESCAPE) = range(10)

    def __init__(self):
        self.tries = 0
        self.last_avoid = 0
        self.dist_l = MAX_DIST
        self.dist_r = MAX_DIST
        self.turn_time = 0
        self.reset()

    def reset(self):
        self.count = 0
        self.dist = MAX_DIST
        self.last_ping = time.ticks_ms()
        self._enter(self.DRIVE)

    def _enter(self, state):
        self.state = state
        self.start = time.ticks_ms()

    def _update_obstacle(self, now):
        if time.ticks_diff(now, self.last_ping) < PING_INTERVAL:
            return
        self.last_ping = now
        self.dist = read_distance_cm()
        if self.dist < OBSTACLE_DIST:
            self.count = min(self.count + 1, 255)
        else:
            self.count = 0

    def _drive_speed(self):
        if self.dist >= SLOW_DIST:
            return SPEED_BASE
        ratio = (self.dist - OBSTACLE_DIST) / (SLOW_DIST - OBSTACLE_DIST)
        ratio = clamp(ratio, 0.0, 1.0)
        return int(SPEED_SLOW + (SPEED_BASE - SPEED_SLOW) * ratio)

    def run(self):
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self.start)
        st = self.state

        if st == self.DRIVE:
            self._update_obstacle(now)
            if self.count >= CONFIRM_COUNT:
                if time.ticks_diff(now, self.last_avoid) < STUCK_WINDOW:
                    self.tries += 1
                else:
                    self.tries = 0
                self.last_avoid = now
                stop_robot()
                self._enter(self.BRAKE)
            else:
                s = self._drive_speed()
                drive(s, s)

        elif st == self.BRAKE:
            if elapsed >= T_BRAKE:
                drive(-SPEED_BASE, -SPEED_BASE)
                self._enter(self.BACK)

        elif st == self.BACK:
            if elapsed >= T_BACK:
                stop_robot()
                self._enter(self.PAUSE)

        elif st == self.PAUSE:
            if elapsed >= T_PAUSE:
                pivot(-1, SPEED_TURN)
                self._enter(self.TURN_L)

        elif st == self.TURN_L:
            if elapsed >= T_SCAN:
                stop_robot()
                self._enter(self.MEAS_L)

        elif st == self.MEAS_L:
            if elapsed >= T_PAUSE:
                self.dist_l = read_distance_median()
                pivot(1, SPEED_TURN)
                self._enter(self.TURN_R)

        elif st == self.TURN_R:
            if elapsed >= 2 * T_SCAN:
                stop_robot()
                self._enter(self.MEAS_R)

        elif st == self.MEAS_R:
            if elapsed >= T_PAUSE:
                self.dist_r = read_distance_median()
                self._decide()

        elif st == self.TURN_BEST:
            if elapsed >= self.turn_time:
                stop_robot()
                self.reset()

        elif st == self.ESCAPE:
            if elapsed >= T_ESCAPE:
                stop_robot()
                self.tries = 0
                self.reset()

    def _decide(self):
        left_blocked = self.dist_l < CLEAR_DIST
        right_blocked = self.dist_r < CLEAR_DIST

        if (left_blocked and right_blocked) or self.tries >= STUCK_TRIES:
            pivot(1, SPEED_TURN)
            self._enter(self.ESCAPE)
            return

        if self.dist_r >= self.dist_l:
            pivot(1, SPEED_TURN)
            self.turn_time = T_SCAN
        else:
            pivot(-1, SPEED_TURN)
            self.turn_time = 3 * T_SCAN
        self._enter(self.TURN_BEST)


MANUAL_CMDS = {
    "F": (1, 0), "B": (-1, 0), "L": (0, -1), "R": (0, 1),
    "Q": (1, -1), "E": (1, 1), "S": (0, 0),
}


def run_manual(cmd):
    fwd, turn = MANUAL_CMDS.get(cmd, (0, 0))
    if fwd == 0 and turn == 0:
        stop_robot()
    elif fwd == 0:
        pivot(turn, SPEED_MANUAL_TURN)
    else:
        l = r = fwd * SPEED_MANUAL
        if turn > 0:
            r //= 3
        elif turn < 0:
            l //= 3
        drive(l, r)


class LineFollower:
    FOLLOW, PIVOT = 0, 1

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = self.FOLLOW
        self.last_error = 0.0
        self.integral = 0.0
        self.d_term = 0.0
        self.last_side = 0
        self.pivot_dir = 0
        self.pivot_start = 0
        self.last_l = 0
        self.last_r = 0
        self.lost_since = None
        self.last_run = time.ticks_ms()

    def run(self):
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self.last_run)
        if elapsed < LF_INTERVAL:
            return
        dt = min(elapsed / 1000.0, 0.05)
        self.last_run = now

        s = [sensor.value() == LINE_DETECTED for sensor in line_sensors]
        active = sum(s)
        total = sum(LF_WEIGHT[i] for i in range(5) if s[i])

        if self.state == self.PIVOT:
            if s[2]:
                self.state = self.FOLLOW
                self.integral = self.d_term = self.last_error = 0.0
            elif time.ticks_diff(now, self.pivot_start) > LF_PIVOT_TIMEOUT:
                self.state = self.FOLLOW
            else:
                pivot(self.pivot_dir, LF_PIVOT_SPEED)
                return

        if active == 0:
            if self.lost_since is None:
                self.lost_since = now
            lost = time.ticks_diff(now, self.lost_since)

            if lost < LF_GAP_HOLD:
                drive(self.last_l, self.last_r)
            elif self.last_side == 0:
                if lost < 800:
                    drive(LF_SPEED_MIN, LF_SPEED_MIN)
                else:
                    stop_robot()
            elif lost > LF_LOST_TIMEOUT:
                stop_robot()
            else:
                t = lost - LF_GAP_HOLD
                d = self.last_side
                if t >= LF_SWEEP_MS:
                    flip = ((t - LF_SWEEP_MS) // (2 * LF_SWEEP_MS)) & 1
                    d = self.last_side if flip else -self.last_side
                pivot(d, LF_PIVOT_SPEED)
            return
        self.lost_since = None

        if active == 5:
            self.integral = self.d_term = self.last_error = 0.0
            self.last_l = self.last_r = LF_SPEED_MIN
            drive(LF_SPEED_MIN, LF_SPEED_MIN)
            return

        if s[0] and not s[1] and not s[2]:
            self.state, self.pivot_dir, self.last_side = self.PIVOT, -1, -1
            self.pivot_start = now
            pivot(-1, LF_PIVOT_SPEED)
            return
        if s[4] and not s[3] and not s[2]:
            self.state, self.pivot_dir, self.last_side = self.PIVOT, 1, 1
            self.pivot_start = now
            pivot(1, LF_PIVOT_SPEED)
            return

        error = total / active

        self.integral += LF_KI * error * dt
        self.integral = clamp(self.integral, -LF_I_LIMIT, LF_I_LIMIT)

        raw_d = (error - self.last_error) / dt
        self.d_term = 0.7 * self.d_term + 0.3 * raw_d
        self.last_error = error

        correction = LF_KP * error + self.integral + LF_KD * self.d_term

        if error < -0.5:
            self.last_side = -1
        elif error > 0.5:
            self.last_side = 1

        ratio = abs(error) / 4.0
        base = LF_SPEED_MAX - int((LF_SPEED_MAX - LF_SPEED_MIN) * ratio)

        l = clamp(int(base + correction), -150, 255)
        r = clamp(int(base - correction), -150, 255)
        self.last_l, self.last_r = l, r
        drive(l, r)


mode = 0
manual_cmd = "S"
last_cmd_ms = 0
avoider = ObstacleAvoider()
follower = LineFollower()


def set_mode(m):
    global mode, manual_cmd
    if m == mode:
        return
    mode = m
    manual_cmd = "S"
    stop_robot()
    avoider.tries = 0
    avoider.reset()
    follower.reset()
    print("Che do:", m)


INDEX_HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>XeRobot</title>
<style>
  body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px;text-align:center;
       user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;touch-action:manipulation}
  h2{margin:8px 0}
  button{font-size:18px;border:0;border-radius:12px;background:#333;color:#eee;touch-action:none}
  .modes{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;max-width:360px;margin:10px auto}
  .modes button{padding:16px 0}
  .modes button.on{background:#1e88e5}
  .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:360px;margin:18px auto}
  .pad button{padding:30px 0;font-size:26px}
  .pad button.hold{background:#43a047}
  .stop{background:#c62828 !important}
  #st{margin-top:8px;font-size:14px;color:#aaa}
</style>
</head>
<body>
<h2>XeRobot ESP32</h2>
<div class="modes">
  <button data-m="0">0 · Dừng</button>
  <button data-m="1">1 · Lái tay</button>
  <button data-m="2">2 · Né vật cản</button>
  <button data-m="3">3 · Dò line</button>
</div>
<div class="pad">
  <button data-c="Q">&#8598;</button>
  <button data-c="F">&#8593;</button>
  <button data-c="E">&#8599;</button>
  <button data-c="L">&#8592;</button>
  <button data-c="S" class="stop">&#9632;</button>
  <button data-c="R">&#8594;</button>
  <span></span>
  <button data-c="B">&#8595;</button>
  <span></span>
</div>
<div id="st">Đang kết nối...</div>
<script>
let timer = null;
function send(c){ fetch('/cmd?c=' + c).catch(()=>{}); }
function press(c){ send(c); clearInterval(timer); timer = setInterval(()=>send(c), 120); }
function release(){ if (timer === null) return; clearInterval(timer); timer = null; send('S'); }

document.querySelectorAll('.pad button').forEach(b => {
  const c = b.dataset.c;
  if (c === 'S') { b.addEventListener('pointerdown', e => { e.preventDefault(); release(); send('S'); }); return; }
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.classList.add('hold'); press(c); });
  ['pointerup','pointercancel','pointerleave'].forEach(ev =>
    b.addEventListener(ev, () => { b.classList.remove('hold'); release(); }));
});
document.addEventListener('contextmenu', e => e.preventDefault());

document.querySelectorAll('.modes button').forEach(b => {
  b.onclick = () => fetch('/mode?m=' + b.dataset.m).then(refresh).catch(()=>{});
});

function refresh(){
  fetch('/status').then(r => r.json()).then(j => {
    document.querySelectorAll('.modes button').forEach(b =>
      b.classList.toggle('on', Number(b.dataset.m) === j.mode));
    document.getElementById('st').textContent = 'Đã kết nối · Chế độ ' + j.mode;
  }).catch(() => { document.getElementById('st').textContent = 'Mất kết nối'; });
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
"""
INDEX_BYTES = INDEX_HTML.encode("utf-8")


def handle_request(route, args):
    global manual_cmd, last_cmd_ms

    if route == "/cmd":
        c = args.get("c", "S")[:1].upper()
        if c in MANUAL_CMDS:
            manual_cmd = c
            last_cmd_ms = time.ticks_ms()
        return "text/plain", b"ok"

    if route == "/mode":
        try:
            set_mode(clamp(int(args.get("m", "0")), 0, 3))
        except ValueError:
            pass
        return "text/plain", b"ok"

    if route == "/status":
        return "application/json", ('{"mode": %d}' % mode).encode()

    if route == "/":
        return "text/html; charset=utf-8", INDEX_BYTES

    return None, None


class WebServer:
    def __init__(self, handler, port=80):
        self.handler = handler
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", port))
        self.srv.listen(5)
        self.srv.setblocking(False)
        self.pending = []

    def poll(self):
        for _ in range(4):
            try:
                conn, _addr = self.srv.accept()
            except OSError:
                break
            conn.setblocking(False)
            self.pending.append([conn, time.ticks_ms(), b""])

        still = []
        for item in self.pending:
            conn, t0, buf = item
            done = False
            try:
                chunk = conn.recv(512)
                if chunk:
                    buf += chunk
                    item[2] = buf
                    if b"\r\n\r\n" in buf:
                        self._respond(conn, buf)
                        done = True
                else:
                    done = True
            except OSError:
                pass
            if not done and time.ticks_diff(time.ticks_ms(), t0) > 1500:
                done = True
            if done:
                try:
                    conn.close()
                except OSError:
                    pass
            else:
                still.append(item)
        self.pending = still

    def _respond(self, conn, raw):
        try:
            line = raw.split(b"\r\n", 1)[0].decode()
            parts = line.split(" ")
            target = parts[1] if len(parts) > 1 else "/"
            route, _, query = target.partition("?")
            args = {}
            for kv in query.split("&"):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    args[k] = v
            ctype, body = self.handler(route, args)
            conn.settimeout(1)
            if ctype is None:
                conn.sendall(b"HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\n")
            else:
                head = ("HTTP/1.0 200 OK\r\nContent-Type: %s\r\n"
                        "Content-Length: %d\r\nCache-Control: no-store\r\n"
                        "Connection: close\r\n\r\n" % (ctype, len(body)))
                conn.sendall(head.encode())
                conn.sendall(body)
        except Exception as e:
            print("HTTP loi:", e)


def start_wifi():
    if STA_SSID:
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        sta.connect(STA_SSID, STA_PASS)
        print("Dang ket noi WiFi...")
        t0 = time.ticks_ms()
        while not sta.isconnected() and time.ticks_diff(time.ticks_ms(), t0) < 10000:
            time.sleep_ms(250)
        if sta.isconnected():
            try:
                sta.config(pm=network.WLAN.PM_NONE)
            except Exception:
                pass
            return sta.ifconfig()[0]
        print("Khong ket noi duoc, chuyen sang tu phat WiFi")
        sta.active(False)

    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASS, authmode=network.AUTH_WPA_WPA2_PSK)
    while not ap.active():
        time.sleep_ms(50)
    return ap.ifconfig()[0]


def main():
    global manual_cmd
    stop_robot()
    ip = start_wifi()
    web = WebServer(handle_request)
    print("San sang! Mo trinh duyet: http://%s" % ip)

    while True:
        web.poll()

        if mode == 0:
            stop_robot()
        elif mode == 1:
            if time.ticks_diff(time.ticks_ms(), last_cmd_ms) > MANUAL_TIMEOUT:
                manual_cmd = "S"
            run_manual(manual_cmd)
        elif mode == 2:
            avoider.run()
        elif mode == 3:
            follower.run()

        time.sleep_ms(1)


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    stop_robot()
