package main

import (
	"encoding/json"
	"fmt"
	"os"
	"strings"
)

// WriteHTML 将回放数据内嵌进单文件 HTML 页面并写出。
// 页面不依赖任何第三方库、不发起任何网络请求，可直接用浏览器打开。
func WriteHTML(path string, data ReplayData) error {
	js, err := json.Marshal(data)
	if err != nil {
		return fmt.Errorf("序列化回放数据失败: %w", err)
	}
	// JSON 中不会出现 "</script>"（数据全为数值与受控字符串），直接替换占位符即可。
	page := strings.Replace(htmlTemplate, "__DATA__", string(js), 1)
	return os.WriteFile(path, []byte(page), 0o644)
}

const htmlTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>多目标航迹关联与生命周期回放</title>
<style>
  :root { --bg:#0f1420; --panel:#171e2e; --ink:#e6ebf5; --muted:#8b96ad;
          --grid:#222b40; --obs:#ffffff; --born:#3ddc84; --died:#ff5d5d;
          --recovered:#5ab8ff; --occluded:#ffb454; --tentative:#9aa5bb; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--ink);
         font:14px/1.5 -apple-system,"PingFang SC","Microsoft YaHei",sans-serif; }
  header { padding:14px 20px 6px; }
  header h1 { font-size:17px; margin:0 0 4px; }
  header .sub { color:var(--muted); font-size:12px; }
  .wrap { display:flex; gap:14px; padding:12px 20px 20px; align-items:flex-start; flex-wrap:wrap; }
  .stage { background:var(--panel); border:1px solid #232d44; border-radius:10px; padding:10px; }
  canvas { display:block; border-radius:6px; background:#0b101b; }
  .side { width:320px; display:flex; flex-direction:column; gap:12px; }
  .card { background:var(--panel); border:1px solid #232d44; border-radius:10px; padding:12px 14px; }
  .card h2 { font-size:12px; color:var(--muted); margin:0 0 8px; font-weight:600;
             letter-spacing:.08em; text-transform:uppercase; }
  .controls { display:flex; align-items:center; gap:8px; flex-wrap:wrap; }
  button { background:#24304a; color:var(--ink); border:1px solid #33415f;
           border-radius:6px; padding:5px 12px; cursor:pointer; font-size:13px; }
  button:hover { background:#2d3c5c; }
  input[type=range] { flex:1; min-width:110px; accent-color:#5b8cff; }
  select { background:#24304a; color:var(--ink); border:1px solid #33415f; border-radius:6px; padding:4px 6px; }
  label.chk { color:var(--muted); font-size:12px; display:flex; align-items:center; gap:5px; }
  .stat { display:flex; justify-content:space-between; font-size:13px; padding:2px 0; }
  .stat b { font-variant-numeric:tabular-nums; }
  .legend { display:grid; grid-template-columns:1fr 1fr; gap:4px 10px; font-size:12px; color:var(--muted); }
  .legend i { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:6px; }
  .events { max-height:260px; overflow-y:auto; font-size:12px; }
  .ev { display:flex; gap:8px; padding:3px 6px; border-radius:5px; cursor:pointer; align-items:baseline; }
  .ev:hover { background:#20293f; }
  .ev .fr { color:var(--muted); min-width:34px; font-variant-numeric:tabular-nums; }
  .ev .tag { min-width:56px; font-weight:600; }
  .ev.born .tag { color:var(--born); } .ev.died .tag, .ev.dropped .tag { color:var(--died); }
  .ev.recovered .tag { color:var(--recovered); } .ev.occluded .tag { color:var(--occluded); }
  .ev.tentative .tag { color:var(--tentative); }
  .ev.now { outline:1px solid #33415f; background:#1d2740; }
</style>
</head>
<body>
<header>
  <h1>多目标航迹关联与生命周期回放</h1>
  <div class="sub">全局最优关联（匈牙利算法 + 虚拟行列，允许不关联） · 马氏距离门控 · 试探/确认/滑行/消亡生命周期</div>
</header>
<div class="wrap">
  <div class="stage"><canvas id="cv" width="940" height="680"></canvas></div>
  <div class="side">
    <div class="card">
      <h2>回放控制</h2>
      <div class="controls">
        <button id="play">▶ 播放</button>
        <button id="prev">‹</button>
        <button id="next">›</button>
        <select id="speed">
          <option value="0.5">0.5×</option>
          <option value="1" selected>1×</option>
          <option value="2">2×</option>
          <option value="4">4×</option>
        </select>
      </div>
      <div class="controls" style="margin-top:8px">
        <input type="range" id="seek" min="0" value="0">
        <span id="frlab" style="font-size:12px;color:var(--muted)"></span>
      </div>
      <div class="controls" style="margin-top:6px">
        <label class="chk"><input type="checkbox" id="trails" checked> 显示轨迹尾迹</label>
        <label class="chk"><input type="checkbox" id="links" checked> 显示关联连线</label>
      </div>
    </div>
    <div class="card">
      <h2>统计</h2>
      <div id="stats"></div>
    </div>
    <div class="card">
      <h2>图例</h2>
      <div class="legend">
        <span><i style="background:var(--obs)"></i>观测点</span>
        <span><i style="background:var(--tentative)"></i>试探航迹</span>
        <span><i style="background:var(--born)"></i>新生（确认）</span>
        <span><i style="background:var(--occluded)"></i>遮挡滑行</span>
        <span><i style="background:var(--recovered)"></i>遮挡恢复</span>
        <span><i style="background:var(--died)"></i>消亡/丢弃</span>
      </div>
    </div>
    <div class="card">
      <h2>事件日志（点击跳转）</h2>
      <div class="events" id="log"></div>
    </div>
  </div>
</div>
<script>
"use strict";
const DATA = __DATA__;
const frames = DATA.frames;
const W = DATA.world[0], H = DATA.world[1];
const cv = document.getElementById("cv");
const ctx = cv.getContext("2d");
const pad = 34;
const sc = Math.min((cv.width - 2*pad) / W, (cv.height - 2*pad) / H);
const ox = (cv.width - W*sc) / 2, oy = (cv.height - H*sc) / 2;
const X = x => ox + x*sc, Y = y => cv.height - (oy + y*sc);

const PALETTE = ["#5b8cff","#ff8c5b","#c86bff","#4fd6c4","#ffd45b","#ff5ba8",
                 "#7bd65b","#8a9bff","#ff6b6b","#5bd6a0"];
const tColor = id => PALETTE[(id-1) % PALETTE.length];
const EVNAME = {tentative:"试探", born:"新生", occluded:"遮挡", recovered:"恢复",
                died:"消亡", dropped:"丢弃"};

let cur = 0, playing = false, timer = null;

const $ = id => document.getElementById(id);
$("seek").max = frames.length - 1;

// 统计面板
(function(){
  const s = DATA.stats;
  const rows = [
    ["总帧数", s.frames],
    ["新生航迹", s.born],
    ["消亡航迹", s.died],
    ["遮挡次数", s.occlusions],
    ["遮挡恢复", s.recovered],
    ["身份保持率", (s.identityAccuracy*100).toFixed(1) + "%"],
    ["ID 切换次数", s.idSwitches],
    ["门限 χ²", DATA.config.gate2],
    ["确认阈值", DATA.config.confirmHits + " 帧"],
    ["消亡阈值", DATA.config.maxMisses + " 帧"],
  ];
  $("stats").innerHTML = rows.map(r =>
    '<div class="stat"><span>'+r[0]+'</span><b>'+r[1]+'</b></div>').join("");
})();

// 事件日志
const logEl = $("log");
const evRows = [];
(function(){
  frames.forEach((f, fi) => {
    f.events.forEach(ev => {
      const div = document.createElement("div");
      div.className = "ev " + ev.type;
      div.dataset.frame = fi;
      div.innerHTML = '<span class="fr">#'+f.frame+'</span><span class="tag">'+
        EVNAME[ev.type]+'</span><span>航迹 T'+ev.track+'</span>';
      div.onclick = () => { seek(fi); };
      logEl.appendChild(div);
      evRows.push(div);
    });
  });
})();

function trackHist(id, upto) {
  const pts = [];
  for (let f = 0; f <= upto; f++) {
    const t = frames[f].tracks.find(t => t.id === id);
    if (t) pts.push(t);
  }
  return pts;
}

function draw() {
  const fr = frames[cur];
  ctx.clearRect(0, 0, cv.width, cv.height);

  // 网格与坐标轴标注
  ctx.strokeStyle = "#1b2337";
  ctx.lineWidth = 1;
  ctx.font = "10px sans-serif";
  for (let gx = 0; gx <= W; gx += 10) {
    ctx.beginPath(); ctx.moveTo(X(gx), Y(0)); ctx.lineTo(X(gx), Y(H)); ctx.stroke();
    ctx.fillStyle = "#4c586f"; ctx.fillText(gx, X(gx)-6, Y(0)+14);
  }
  for (let gy = 0; gy <= H; gy += 10) {
    ctx.beginPath(); ctx.moveTo(X(0), Y(gy)); ctx.lineTo(X(W), Y(gy)); ctx.stroke();
    ctx.fillStyle = "#4c586f"; ctx.fillText(gy, 8, Y(gy)+3);
  }

  // 轨迹尾迹
  if ($("trails").checked) {
    fr.tracks.forEach(t => {
      const hist = trackHist(t.id, cur).slice(-25);
      if (hist.length < 2) return;
      ctx.strokeStyle = tColor(t.id) + "55"; ctx.lineWidth = 1.5;
      ctx.beginPath();
      hist.forEach((p, i) => { i ? ctx.lineTo(X(p.x), Y(p.y)) : ctx.moveTo(X(p.x), Y(p.y)); });
      ctx.stroke();
    });
  }

  // 观测点与关联连线
  fr.obs.forEach(o => {
    if ($("links").checked && o.track >= 0) {
      const t = fr.tracks.find(t => t.id === o.track);
      if (t) {
        ctx.strokeStyle = tColor(o.track) + "66"; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(X(o.x), Y(o.y)); ctx.lineTo(X(t.x), Y(t.y)); ctx.stroke();
      }
    }
    ctx.fillStyle = "#ffffff";
    ctx.beginPath(); ctx.arc(X(o.x), Y(o.y), 3, 0, 7); ctx.fill();
  });

  // 航迹
  fr.tracks.forEach(t => {
    const x = X(t.x), y = Y(t.y);
    if (t.state === "tentative") {
      ctx.strokeStyle = "#9aa5bb"; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, 5, 0, 7); ctx.stroke();
      ctx.fillStyle = "#9aa5bb"; ctx.font = "10px sans-serif";
      ctx.fillText("t"+t.id, x+8, y-6);
      return;
    }
    ctx.fillStyle = tColor(t.id);
    ctx.beginPath(); ctx.arc(x, y, 6, 0, 7); ctx.fill();
    if (t.state === "coasting") {
      ctx.strokeStyle = "#ffb454"; ctx.lineWidth = 2; ctx.setLineDash([4,3]);
      ctx.beginPath(); ctx.arc(x, y, 10, 0, 7); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = "#ffb454"; ctx.font = "10px sans-serif";
      ctx.fillText("漏检 " + t.misses, x+12, y+14);
    }
    ctx.fillStyle = tColor(t.id); ctx.font = "bold 11px sans-serif";
    ctx.fillText("T"+t.id, x+9, y-8);
  });

  // 本帧事件高亮
  fr.events.forEach(ev => {
    const x = X(ev.x), y = Y(ev.y);
    ctx.lineWidth = 2;
    if (ev.type === "born") {
      ctx.strokeStyle = "#3ddc84";
      ctx.beginPath(); ctx.arc(x, y, 13, 0, 7); ctx.stroke();
      ctx.fillStyle = "#3ddc84"; ctx.font = "bold 11px sans-serif"; ctx.fillText("新生", x+15, y-12);
    } else if (ev.type === "died" || ev.type === "dropped") {
      ctx.strokeStyle = "#ff5d5d";
      ctx.beginPath();
      ctx.moveTo(x-8, y-8); ctx.lineTo(x+8, y+8);
      ctx.moveTo(x+8, y-8); ctx.lineTo(x-8, y+8); ctx.stroke();
      ctx.fillStyle = "#ff5d5d"; ctx.font = "bold 11px sans-serif";
      ctx.fillText(ev.type === "died" ? "消亡" : "丢弃", x+11, y-10);
    } else if (ev.type === "recovered") {
      ctx.strokeStyle = "#5ab8ff";
      ctx.beginPath(); ctx.arc(x, y, 13, 0, 7); ctx.stroke();
      ctx.beginPath(); ctx.arc(x, y, 17, 0, 7); ctx.stroke();
      ctx.fillStyle = "#5ab8ff"; ctx.font = "bold 11px sans-serif"; ctx.fillText("恢复", x+19, y-14);
    } else if (ev.type === "occluded") {
      ctx.strokeStyle = "#ffb454"; ctx.setLineDash([3,3]);
      ctx.beginPath(); ctx.arc(x, y, 13, 0, 7); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = "#ffb454"; ctx.font = "bold 11px sans-serif"; ctx.fillText("遮挡", x+15, y-12);
    }
  });

  $("frlab").textContent = "帧 " + fr.frame + " / " + frames.length;
  $("seek").value = cur;
  evRows.forEach(d => d.classList.toggle("now", +d.dataset.frame === cur));
}

function seek(i) {
  cur = Math.max(0, Math.min(frames.length - 1, i));
  draw();
}
function step(d) { seek(cur + d); }

function setPlay(p) {
  playing = p;
  $("play").textContent = playing ? "⏸ 暂停" : "▶ 播放";
  if (timer) { clearInterval(timer); timer = null; }
  if (playing) {
    const tick = () => {
      if (cur >= frames.length - 1) { setPlay(false); return; }
      step(1);
    };
    timer = setInterval(tick, 160 / parseFloat($("speed").value));
  }
}
$("play").onclick = () => setPlay(!playing);
$("prev").onclick = () => step(-1);
$("next").onclick = () => step(1);
$("seek").oninput = e => seek(+e.target.value);
$("speed").onchange = () => { if (playing) setPlay(true); };
document.addEventListener("keydown", e => {
  if (e.key === " ") { e.preventDefault(); setPlay(!playing); }
  if (e.key === "ArrowLeft") step(-1);
  if (e.key === "ArrowRight") step(1);
});
$("trails").onchange = draw;
$("links").onchange = draw;
draw();
</script>
</body>
</html>
`
