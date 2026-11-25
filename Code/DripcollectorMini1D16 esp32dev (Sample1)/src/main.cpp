/*
 * ROVER CONTROL v14 - REAR WHEEL DRIVE + CONVEYOR (FWD/REV) + JOYSTICK + GEARS  (ESP32)
 *
 *   M1 Rear Left : GPIO27 -> L298N #1 IN1   GPIO26 -> L298N #1 IN2
 *   M2 Rear Right: GPIO25 -> L298N #1 IN3   GPIO33 -> L298N #1 IN4
 *   Wheel enable : GPIO12 -> L298N #1 ENA + ENB (tied together, jumpers removed)
 *   M3 Conveyor  : GPIO32 -> L298N #2 IN1   GPIO14 -> L298N #2 IN2  (EN jumper ON)
 *   Common GND for everything.
 *
 *   Wheels use PWM on the IN pins. Gear (N,1-4) caps wheel speed.
 *   EN is HIGH only while a wheel is being driven.
 *
 *   NOTE: GPIO12 is a strapping pin. If the board won't boot, unplug the
 *   GPIO12 wire while powering up, or add a 10k resistor from GPIO12 to GND.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------------- CONFIG ----------------
const char*   WIFI_SSID    = "MC FARM";
const char*   WIFI_PASS    = "9876543210";
const uint8_t WIFI_CHANNEL = 6;
const uint8_t MAX_CLIENTS  = 4;

// PWM pins in order: RL_A, RL_B, RR_A, RR_B  (index = LEDC channel on core 2.x)
const uint8_t PWM_PINS[4] = { 27, 26, 25, 33 };

// One shared enable pin for both wheel channels (ENA + ENB)
const uint8_t EN_PIN = 12;

// Conveyor
const uint8_t CONV_A = 32;   // forward
const uint8_t CONV_B = 14;   // reverse

// Flip to -1 if that motor spins the wrong way on "forward"
const int8_t WHEEL_POLARITY[2] = { 1, -1 };
const int8_t CONV_POLARITY     = 1;

// Gear speed caps in percent. Index 0 = Neutral, 1..4 = gears.
const uint8_t GEAR_MAX[5] = { 0, 30, 50, 75, 100 };
const uint8_t DEFAULT_GEAR = 2;

// Lowest PWM duty (0-255) that still turns a wheel.
const uint8_t MIN_PWM = 70;

const unsigned long MOVE_TIMEOUT_MS = 700;
// ----------------------------------------

struct State {
    bool systemOn = false;
    int  req[2]   = { 0, 0 };   // requested wheel speed -100..100 (RL, RR)
    int8_t conv   = 0;          // conveyor -1, 0, 1
    uint8_t gear  = DEFAULT_GEAR;
    unsigned long lastMoveMs = 0;
} st;

WebServer server(80);

// ---------------- PWM ----------------
void pwmOut(uint8_t idx, uint8_t duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PWM_PINS[idx], duty);
#else
    ledcWrite(idx, duty);
#endif
}

void pwmInit()
{
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(PWM_PINS[i], OUTPUT);
        digitalWrite(PWM_PINS[i], LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(PWM_PINS[i], 5000, 8);
#else
        ledcSetup(i, 5000, 8);
        ledcAttachPin(PWM_PINS[i], i);
#endif
        pwmOut(i, 0);
    }
}

// ---------------- HARDWARE ----------------
int gearScale(int req)
{
    if (req == 0) return 0;
    int v = req * (int)GEAR_MAX[st.gear] / 100;
    if (v == 0 && GEAR_MAX[st.gear] > 0) v = (req > 0) ? 1 : -1;
    return v;
}

void applyMotors()
{
    bool anyWheel = false;
    for (uint8_t i = 0; i < 2; i++) {
        int v = gearScale(st.req[i]) * WHEEL_POLARITY[i];
        uint8_t duty = 0;
        if (v != 0) { duty = map(abs(v), 1, 100, MIN_PWM, 255); anyWheel = true; }
        pwmOut(i * 2,     v > 0 ? duty : 0);
        pwmOut(i * 2 + 1, v < 0 ? duty : 0);
    }
    // Shared enable: HIGH only while at least one wheel is driven
    digitalWrite(EN_PIN, anyWheel ? HIGH : LOW);

    int c = st.conv * CONV_POLARITY;
    digitalWrite(CONV_A, c > 0 ? HIGH : LOW);
    digitalWrite(CONV_B, c < 0 ? HIGH : LOW);
}

bool wheelsMoving() { return st.req[0] != 0 || st.req[1] != 0; }
bool anyMoving()    { return wheelsMoving() || st.conv != 0; }

void stopWheels()
{
    st.req[0] = st.req[1] = 0;
    applyMotors();
}

void stopAll()
{
    st.req[0] = st.req[1] = 0;
    st.conv = 0;
    applyMotors();
}

void setWheels(int l, int r)
{
    st.req[0] = st.systemOn ? l : 0;
    st.req[1] = st.systemOn ? r : 0;
    if (wheelsMoving()) st.lastMoveMs = millis();
    applyMotors();
}

void setConveyor(int8_t d)
{
    st.conv = st.systemOn ? d : 0;
    applyMotors();
}

void setGear(uint8_t g)
{
    st.gear = g;
    applyMotors();
}

void setSystem(bool on)
{
    st.systemOn = on;
    stopAll();
}

// ---------------- WEB UI ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ROVER</title>
<style>
*{box-sizing:border-box}
body{background:#111;color:#fff;font-family:Arial,sans-serif;text-align:center;margin:0;padding:16px;
     -webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
h1{font-size:26px;margin:6px 0}
h2{font-size:15px;color:#aaa;margin:22px 0 6px;letter-spacing:2px}
button{width:104px;height:58px;margin:5px;border:0;border-radius:14px;font-size:16px;
       font-weight:bold;color:#fff;touch-action:none}
button:active{filter:brightness(1.4)}
.mv{background:#2196F3}.st{background:#F44336}.on{background:#4CAF50}
.off{background:#555}.cv{background:#7E57C2}
.mt{background:#7E57C2;width:80px;height:48px;margin:3px;font-size:13px}
.gr{background:#FF9800;width:58px;height:58px;margin:4px;font-size:20px;border-radius:50%}
.gr.n{background:#555}
.lit{outline:3px solid #fff;filter:brightness(1.25)}
#bar{display:flex;gap:8px;justify-content:center;margin:8px 0;flex-wrap:wrap}
.pill{padding:6px 12px;border-radius:20px;background:#333;font-size:13px}
.pill.a{background:#4CAF50}
#conn{color:#f66;font-size:12px;height:16px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:420px;margin:0 auto}
.box{background:#1c1c1c;border-radius:12px;padding:6px}
.box b{font-size:13px;color:#ccc}
small{color:#888}
#joy{width:240px;height:240px;border-radius:50%;background:radial-gradient(#2a2a2a,#181818);
     border:3px solid #444;margin:10px auto;position:relative;touch-action:none}
#joy:before,#joy:after{content:"";position:absolute;background:#333}
#joy:before{left:50%;top:10px;bottom:10px;width:2px;margin-left:-1px}
#joy:after{top:50%;left:10px;right:10px;height:2px;margin-top:-1px}
#knob{width:90px;height:90px;border-radius:50%;background:#2196F3;position:absolute;
      left:75px;top:75px;box-shadow:0 4px 12px #000;pointer-events:none;z-index:2}
#joy.act #knob{background:#64B5F6}
#lr{font-size:13px;color:#aaa;height:18px}
</style></head><body>
<h1>ROVER CONTROL</h1>
<div id="conn"></div>
<div id="bar">
  <span id="p_sys" class="pill">SYSTEM</span>
  <span id="p_conv" class="pill">CONVEYOR</span>
  <span id="p_gear" class="pill">GEAR 2</span>
</div>

<h2>SYSTEM</h2>
<button class="on" onclick="cmd('systemon')">ON</button>
<button class="off" onclick="cmd('systemoff')">OFF</button>

<h2>GEAR</h2>
<button id="g0" class="gr n" onclick="gear(0)">N</button><button id="g1" class="gr" onclick="gear(1)">1</button><button id="g2" class="gr" onclick="gear(2)">2</button><button id="g3" class="gr" onclick="gear(3)">3</button><button id="g4" class="gr" onclick="gear(4)">4</button>
<br><small>1 = crawl &nbsp;&middot;&nbsp; 4 = full speed &nbsp;&middot;&nbsp; N = neutral</small>

<h2>CONVEYOR BELT</h2>
<small>Keeps running until you press STOP</small><br>
<button id="cf" class="cv" onclick="conv(1)">FORWARD</button>
<button id="cs" class="off" onclick="conv(0)">STOP</button>
<button id="cr" class="cv" onclick="conv(-1)">REVERSE</button>

<h2>JOYSTICK</h2>
<div id="joy"><div id="knob"></div></div>
<div id="lr">L 0 &nbsp; R 0</div>

<h2>DRIVE BUTTONS (HOLD)</h2>
<div><button id="fwd" class="mv">FORWARD</button></div>
<div>
  <button id="left" class="mv">LEFT</button>
  <button class="st" onclick="stopWheels()">STOP</button>
  <button id="right" class="mv">RIGHT</button>
</div>
<div><button id="rev" class="mv">REVERSE</button></div>

<h2>EACH WHEEL (TAP TO RUN, TAP AGAIN TO STOP)</h2>
<div class="grid">
  <div class="box"><b>Rear Left</b><br>
    <button id="w0r" class="mt" onclick="tog(0,-1)">REV</button><button id="w0f" class="mt" onclick="tog(0,1)">FWD</button></div>
  <div class="box"><b>Rear Right</b><br>
    <button id="w1r" class="mt" onclick="tog(1,-1)">REV</button><button id="w1f" class="mt" onclick="tog(1,1)">FWD</button></div>
</div>

<script>
// wd = requested wheel speeds [left,right], -100..100. The rover applies the gear limit.
var wd=[0,0], hb=null, cd=0, gr=2, busy=false;

function sg(v){ return v>0?1:(v<0?-1:0); }
function anyOn(){ return wd[0]!==0 || wd[1]!==0; }

function pushWd(force){
  if(busy && !force) return;
  busy=true;
  return fetch('/cmd?c=wheels&v='+wd.join(',')).catch(function(){}).then(function(){busy=false;});
}

function setWd(a, immediate){
  wd=[Math.round(a[0]),Math.round(a[1])];
  if(immediate!==false) pushWd(true);
  render();
  if(anyOn()){ if(!hb) hb=setInterval(function(){pushWd(false);},120); }
  else if(hb){ clearInterval(hb); hb=null; }
}
function tog(i,d){ var a=wd.slice(); a[i]=(sg(a[i])===d)?0:d*100; setWd(a); }
function stopWheels(){ setWd([0,0]); fetch('/cmd?c=stopwheels').catch(function(){}); }

function conv(d){ cd=d; render(); fetch('/cmd?c=conv&v='+d).then(poll).catch(function(){}); }
function cmd(q){ return fetch('/cmd?c='+q).then(poll).catch(function(){}); }
function gear(g){ gr=g; render(); fetch('/cmd?c=gear&v='+g).then(poll).catch(function(){}); }

// ---- hold buttons ----
function hold(id,p){
  var b=document.getElementById(id), on=false;
  function s(e){ e.preventDefault(); on=true; setWd([p[0]*100,p[1]*100]); }
  function x(e){ e.preventDefault(); if(on){ on=false; setWd([0,0]); } }
  b.addEventListener('pointerdown',s);
  ['pointerup','pointerleave','pointercancel'].forEach(function(ev){b.addEventListener(ev,x);});
  b.addEventListener('contextmenu',function(e){e.preventDefault();});
}
hold('fwd',[1,1]); hold('rev',[-1,-1]);
hold('left',[-1,1]); hold('right',[1,-1]);

// ---- joystick ----
var jb=document.getElementById('joy'), kn=document.getElementById('knob'), ja=false, first=true;
function joyDrive(x,y){
  var l=y+x, r=y-x;
  var m=Math.max(Math.abs(l),Math.abs(r),1);
  l=l/m*100; r=r/m*100;
  setWd([l,r], first); first=false;
}
function jmove(e){
  var rc=jb.getBoundingClientRect(), R=rc.width/2;
  var dx=e.clientX-(rc.left+R), dy=e.clientY-(rc.top+R);
  var max=R-45, len=Math.sqrt(dx*dx+dy*dy);
  if(len>max){ dx*=max/len; dy*=max/len; }
  kn.style.transform='translate('+dx+'px,'+dy+'px)';
  var x=dx/max, y=-dy/max;
  if(Math.sqrt(x*x+y*y)<0.12){ x=0; y=0; }        // dead zone
  joyDrive(x,y);
}
function jend(e){
  if(!ja) return;
  ja=false; first=true;
  jb.classList.remove('act');
  kn.style.transform='translate(0,0)';
  setWd([0,0]);
}
jb.addEventListener('pointerdown',function(e){
  e.preventDefault(); ja=true; first=true;
  jb.classList.add('act');
  try{ jb.setPointerCapture(e.pointerId); }catch(_){}
  jmove(e);
});
jb.addEventListener('pointermove',function(e){ if(ja){ e.preventDefault(); jmove(e); } });
jb.addEventListener('pointerup',jend);
jb.addEventListener('pointercancel',jend);
jb.addEventListener('lostpointercapture',jend);
jb.addEventListener('contextmenu',function(e){e.preventDefault();});

// ---- display ----
function lit(id,on){ document.getElementById(id).classList.toggle('lit',on); }
function render(){
  var a=sg(wd[0]), b=sg(wd[1]);
  var fromButtons=!ja;
  lit('fwd',fromButtons&&a===1&&b===1);   lit('rev',fromButtons&&a===-1&&b===-1);
  lit('left',fromButtons&&a===-1&&b===1); lit('right',fromButtons&&a===1&&b===-1);
  lit('w0f',fromButtons&&a===1); lit('w0r',fromButtons&&a===-1);
  lit('w1f',fromButtons&&b===1); lit('w1r',fromButtons&&b===-1);
  lit('cf',cd===1); lit('cr',cd===-1); lit('cs',cd===0);
  for(var i=0;i<5;i++) lit('g'+i,gr===i);
  document.getElementById('p_gear').textContent='GEAR '+(gr===0?'N':gr);
  document.getElementById('lr').innerHTML='L '+wd[0]+' &nbsp; R '+wd[1];
}

function poll(){
  return fetch('/status').then(function(r){return r.json();}).then(function(s){
    document.getElementById('conn').textContent='';
    document.getElementById('p_sys').className='pill'+(s.sys?' a':'');
    document.getElementById('p_conv').className='pill'+(s.m[2]!==0?' a':'');
    cd=s.m[2]; gr=s.g;
    if(!s.sys && anyOn()) setWd([0,0]);
    render();
  }).catch(function(){
    document.getElementById('conn').textContent='Connection lost...';
    wd=[0,0]; if(hb){clearInterval(hb);hb=null;} render();
  });
}
setInterval(poll,1000); poll(); render();
</script></body></html>
)rawliteral";

void handleStatus()
{
    String j = "{\"sys\":";   j += st.systemOn ? "true" : "false";
    j += ",\"g\":";           j += st.gear;
    j += ",\"m\":[";
    j += st.req[0]; j += ","; j += st.req[1]; j += ","; j += st.conv;
    j += "]}";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", j);
}

bool parseWheels(const String& v, int& l, int& r)
{
    int comma = v.indexOf(',');
    if (comma <= 0 || comma >= (int)v.length() - 1) return false;
    int a = v.substring(0, comma).toInt();
    int b = v.substring(comma + 1).toInt();
    if (a < -100 || a > 100 || b < -100 || b > 100) return false;
    l = a; r = b;
    return true;
}

void handleCmd()
{
    String c = server.arg("c");

    if      (c == "systemon")   setSystem(true);
    else if (c == "systemoff")  setSystem(false);
    else if (c == "stopwheels") stopWheels();
    else if (c == "stop")       stopAll();
    else if (c == "wheels") {
        int l, r;
        if (!parseWheels(server.arg("v"), l, r)) { server.send(400, "text/plain", "BAD"); return; }
        setWheels(l, r);
    }
    else if (c == "conv") {
        int d = server.arg("v").toInt();
        if (d < -1 || d > 1) { server.send(400, "text/plain", "BAD"); return; }
        setConveyor(d);
    }
    else if (c == "gear") {
        int g = server.arg("v").toInt();
        if (g < 0 || g > 4) { server.send(400, "text/plain", "BAD"); return; }
        setGear(g);
    }
    else { server.send(400, "text/plain", "BAD"); return; }

    server.send(200, "text/plain", "OK");
}

// ---------------- SETUP / LOOP ----------------
void setup()
{
    // Enable LOW first, so the wheel driver stays disabled at boot
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    pinMode(CONV_A, OUTPUT);
    pinMode(CONV_B, OUTPUT);
    digitalWrite(CONV_A, LOW);
    digitalWrite(CONV_B, LOW);
    pwmInit();
    stopAll();

    Serial.begin(115200);
    Serial.println("\nRover ready");

    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_CLIENTS);

    Serial.printf("AP \"%s\"  IP: %s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());

    server.on("/",       []{ server.send_P(200, "text/html", PAGE); });
    server.on("/cmd",    handleCmd);
    server.on("/status", handleStatus);
    server.onNotFound([]{ server.send(404, "text/plain", "Not found"); });
    server.begin();
}

void loop()
{
    server.handleClient();

    // Wheels stop if commands go stale; conveyor keeps running
    if (wheelsMoving() && millis() - st.lastMoveMs > MOVE_TIMEOUT_MS) {
        stopWheels();
    }

    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 500) {
        lastCheck = millis();

        // Nobody connected: stop everything
        if (WiFi.softAPgetStationNum() == 0 && anyMoving()) {
            stopAll();
        }
    }
}