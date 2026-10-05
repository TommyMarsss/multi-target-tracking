package main

import (
	"math/rand"
)

// 仿真世界尺寸（平面直角坐标，单位任意）。
const (
	worldW = 100.0
	worldH = 70.0
)

// gtTarget 描述一个真值目标的分段匀速运动及出现/消失/遮挡区间。
type gtTarget struct {
	id    int
	start Point    // from 帧的位置
	vel   Point    // 每帧速度
	from  int      // 首次出现帧（含）
	to    int      // 最后出现帧（含），-1 表示全程
	gaps  [][2]int // 遮挡区间 [from,to]（含），期间无观测
}

func (g gtTarget) posAt(f int) (Point, bool) {
	if f < g.from || (g.to >= 0 && f > g.to) {
		return Point{}, false
	}
	for _, gap := range g.gaps {
		if f >= gap[0] && f <= gap[1] {
			return Point{}, false
		}
	}
	df := float64(f - g.from)
	return Point{g.start.X + g.vel.X*df, g.start.Y + g.vel.Y*df}, true
}

// simObs 是带真值标签的仿真观测（标签仅用于评估，不进入跟踪器）。
type simObs struct {
	pt Point
	gt int // 真值目标 ID，-1 表示杂波
}

// simulate 生成逐帧观测：每个在场真值目标产生一个加噪观测，另加随机杂波。
func simulate(gts []gtTarget, frames int, measSigma, clutterProb float64, rng *rand.Rand) [][]simObs {
	out := make([][]simObs, frames)
	for f := 0; f < frames; f++ {
		for _, g := range gts {
			p, ok := g.posAt(f)
			if !ok {
				continue
			}
			out[f] = append(out[f], simObs{pt: Point{
				p.X + rng.NormFloat64()*measSigma,
				p.Y + rng.NormFloat64()*measSigma,
			}, gt: g.id})
		}
		if rng.Float64() < clutterProb {
			n := 1 + rng.Intn(2)
			for i := 0; i < n; i++ {
				out[f] = append(out[f], simObs{pt: Point{
					rng.Float64() * worldW,
					rng.Float64() * worldH,
				}, gt: -1})
			}
		}
	}
	return out
}

// Stats 是一段仿真的评估指标。
type Stats struct {
	Frames           int     `json:"frames"`
	Born             int     `json:"born"`
	Died             int     `json:"died"`
	Recovered        int     `json:"recovered"`
	Occlusions       int     `json:"occlusions"`
	IdentityAccuracy float64 `json:"identityAccuracy"` // 航迹身份保持正确率
	IDSwitches       int     `json:"idSwitches"`       // 真值目标被换轨的次数
}

// ReplayData 是 HTML 回放所需的全部数据。
type ReplayData struct {
	World  [2]float64    `json:"world"`
	Config Config        `json:"config"`
	Stats  Stats         `json:"stats"`
	Frames []FrameRecord `json:"frames"`
}

// RunScenario 运行一段仿真并返回回放数据与逐帧真值标签（后者供测试评估）。
func RunScenario(gts []gtTarget, frames int, cfg Config, measSigma, clutterProb float64, seed int64) (ReplayData, [][]int) {
	rng := rand.New(rand.NewSource(seed))
	sim := simulate(gts, frames, measSigma, clutterProb, rng)

	tr := NewTracker(cfg)
	data := ReplayData{World: [2]float64{worldW, worldH}, Config: cfg}
	gtLabels := make([][]int, frames)
	for f := 0; f < frames; f++ {
		obs := make([]Point, len(sim[f]))
		gtLabels[f] = make([]int, len(sim[f]))
		for i, so := range sim[f] {
			obs[i] = so.pt
			gtLabels[f][i] = so.gt
		}
		data.Frames = append(data.Frames, tr.Step(obs))
	}
	data.Stats = evalStats(data.Frames, gtLabels)
	return data, gtLabels
}

// evalStats 统计生命周期事件数量与航迹身份保持正确率。
//
// 身份保持率的计算：先按 (航迹, 真值目标) 关联帧数投票出每条航迹的"主身份"，
// 再统计关联到真值目标的观测中、航迹主身份与观测真值一致的比例。
// ID 切换次数：同一真值目标在相邻两次被关联之间航迹 ID 发生变化的次数。
// 两者都只统计确认航迹：单帧门控漏检产生的孤立试探航迹（下一帧即被丢弃）
// 不代表已建立的身份，不计入身份切换。
func evalStats(recs []FrameRecord, gtLabels [][]int) Stats {
	st := Stats{Frames: len(recs)}
	type pair struct{ t, g int }
	cnt := map[pair]int{}
	lastTrack := map[int]int{} // 真值目标 -> 最近一次关联的确认航迹 ID
	for f, rec := range recs {
		confirmed := map[int]bool{}
		for _, tr := range rec.Tracks {
			if tr.State != "tentative" {
				confirmed[tr.ID] = true
			}
		}
		for _, ev := range rec.Events {
			switch ev.Type {
			case "born":
				st.Born++
			case "died":
				st.Died++
			case "recovered":
				st.Recovered++
			case "occluded":
				st.Occlusions++
			}
		}
		for i, o := range rec.Obs {
			g := gtLabels[f][i]
			if g < 0 || o.Track < 0 || !confirmed[o.Track] {
				continue
			}
			cnt[pair{o.Track, g}]++
			if lt, ok := lastTrack[g]; ok && lt != o.Track {
				st.IDSwitches++
			}
			lastTrack[g] = o.Track
		}
	}
	home := map[int]int{} // 航迹 -> 主身份（真值目标）
	best := map[int]int{}
	for p, c := range cnt {
		if c > best[p.t] {
			best[p.t] = c
			home[p.t] = p.g
		}
	}
	correct, total := 0, 0
	for p, c := range cnt {
		total += c
		if home[p.t] == p.g {
			correct += c
		}
	}
	if total > 0 {
		st.IdentityAccuracy = float64(correct) / float64(total)
	}
	return st
}

// demoScenario 返回演示场景：两目标交叉、一个中途出现/消失的目标、
// 一个被短暂遮挡的目标，全程叠加随机杂波。
func demoScenario() ([]gtTarget, int) {
	return []gtTarget{
		// T0 与 T1 在第 ~38 帧附近交叉，用于验证歧义分配下不发生身份交换。
		{id: 0, start: Point{6, 18}, vel: Point{1.15, 0.34}, from: 0, to: -1},
		{id: 1, start: Point{94, 44}, vel: Point{-1.15, -0.34}, from: 0, to: -1},
		// T2 第 15 帧出现、第 55 帧消失，用于验证新生与消亡判定。
		{id: 2, start: Point{12, 58}, vel: Point{0.85, -0.12}, from: 15, to: 55},
		// T3 第 30–34 帧被遮挡（5 帧，恰在 MaxMisses 阈值内），用于验证遮挡恢复。
		{id: 3, start: Point{78, 8}, vel: Point{-0.55, 0.42}, from: 0, to: -1, gaps: [][2]int{{30, 34}}},
	}, 100
}
