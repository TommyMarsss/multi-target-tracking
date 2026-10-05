package main

// bigCost 是代价矩阵中"不可行匹配"的惩罚值。它必须远大于任何可行代价，
// 使匈牙利算法只有在不存在任何有限完美匹配时才会选择它；
// 由于我们同时提供了虚拟行/列（代价有限），不可行匹配实际上永远不会被选中。
const bigCost = 1e12

// Config 是跟踪器的全部可调参数。
type Config struct {
	// Gate2 是关联门限（平方马氏距离）。取 χ²(2, 0.99) ≈ 9.21：
	// 代价超过门限的观测-航迹对为不可行匹配，绝不强制关联。
	Gate2 float64 `json:"gate2"`
	// ConfirmHits 是试探航迹转为确认航迹所需的连续关联帧数（M/N 逻辑的 M=N）。
	ConfirmHits int `json:"confirmHits"`
	// MaxMisses 是确认航迹允许的最大连续漏检帧数；超过则判定消亡。
	MaxMisses int `json:"maxMisses"`
	// ProcessNoise 是过程噪声谱密度 q（加速度噪声）。
	ProcessNoise float64 `json:"processNoise"`
	// MeasNoise 是观测噪声方差 r²。
	MeasNoise float64 `json:"measNoise"`
}

// DefaultConfig 返回一组对演示场景调好的默认参数。
func DefaultConfig() Config {
	return Config{
		Gate2:        9.21,
		ConfirmHits:  3,
		MaxMisses:    5,
		ProcessNoise: 0.5,
		MeasNoise:    1.0,
	}
}

// Track 是一条航迹。
type Track struct {
	ID       int
	f        kalmanFilter
	hits     int // 连续关联帧数（用于确认判定）
	misses   int // 连续漏检帧数（用于消亡判定）
	state    TrackState
	coasting bool          // 确认航迹处于漏检滑行（遮挡）状态
	coastS   [2][2]float64 // 进入滑行时冻结的新息协方差（遮挡期间门控不膨胀）
}

// Tracker 维护一组航迹，逐帧处理观测。
type Tracker struct {
	cfg    Config
	tracks []*Track
	nextID int
	frame  int
}

// NewTracker 创建跟踪器。
func NewTracker(cfg Config) *Tracker {
	return &Tracker{cfg: cfg, nextID: 1}
}

// Step 处理一帧观测，推进所有航迹，并返回该帧的回放记录（含生命周期事件）。
func (tr *Tracker) Step(obs []Point) FrameRecord {
	tr.frame++
	rec := FrameRecord{Frame: tr.frame}

	// 1. 时间更新：所有航迹外推到当前帧。
	for _, t := range tr.tracks {
		t.f.predict(tr.cfg.ProcessNoise)
	}

	// 2. 全局最优关联（匈牙利算法 + 虚拟行/列，允许不关联）。
	obsForTrack := tr.associate(obs)

	// 3. 量测更新与确认/恢复判定。
	trackForObs := make([]int, len(obs))
	for i := range trackForObs {
		trackForObs[i] = -1
	}
	for ti, t := range tr.tracks {
		oi := obsForTrack[ti]
		if oi < 0 {
			continue
		}
		t.f.update(obs[oi], tr.cfg.MeasNoise)
		t.hits++
		if t.state == Confirmed && t.coasting {
			// 遮挡期间持续外推，漏检未超阈值即重新关联：恢复原航迹，身份不变。
			rec.Events = append(rec.Events, Event{Type: "recovered", TrackID: t.ID, X: t.f.x[0], Y: t.f.x[1]})
		}
		t.misses = 0
		t.coasting = false
		if t.state == Tentative && t.hits >= tr.cfg.ConfirmHits {
			t.state = Confirmed
			rec.Events = append(rec.Events, Event{Type: "born", TrackID: t.ID, X: t.f.x[0], Y: t.f.x[1]})
		}
		trackForObs[oi] = t.ID
	}

	// 4. 未关联航迹的生命周期处理。
	kept := tr.tracks[:0]
	for ti, t := range tr.tracks {
		if obsForTrack[ti] >= 0 {
			kept = append(kept, t)
			continue
		}
		t.hits = 0
		t.misses++
		if t.state == Tentative {
			// 试探航迹要求连续关联，一帧漏检即丢弃——孤立噪声点不会建立航迹。
			rec.Events = append(rec.Events, Event{Type: "dropped", TrackID: t.ID, X: t.f.x[0], Y: t.f.x[1]})
			continue
		}
		if t.misses == 1 {
			t.coasting = true
			// 冻结进入遮挡时的门控协方差：滑行期间 P 持续增长，
			// 若不冻结，门控区域会膨胀到吞掉邻近目标的观测（身份交换的温床）。
			t.coastS = t.f.innovationS(tr.cfg.MeasNoise)
			rec.Events = append(rec.Events, Event{Type: "occluded", TrackID: t.ID, X: t.f.x[0], Y: t.f.x[1]})
		}
		if t.misses > tr.cfg.MaxMisses {
			// 连续漏检超过阈值才判定消亡，单帧漏检不会删除航迹。
			rec.Events = append(rec.Events, Event{Type: "died", TrackID: t.ID, X: t.f.x[0], Y: t.f.x[1]})
			continue
		}
		kept = append(kept, t)
	}
	tr.tracks = kept

	// 5. 未关联观测 -> 创建试探航迹（此时还不算"新航迹"，需连续稳定出现才确认）。
	for oi, z := range obs {
		if trackForObs[oi] >= 0 {
			continue
		}
		t := &Track{ID: tr.nextID, f: newKalman(z, tr.cfg.MeasNoise), hits: 1, state: Tentative}
		tr.nextID++
		tr.tracks = append(tr.tracks, t)
		rec.Events = append(rec.Events, Event{Type: "tentative", TrackID: t.ID, X: z.X, Y: z.Y})
		trackForObs[oi] = t.ID
	}

	// 6. 记录回放快照。
	for i, z := range obs {
		rec.Obs = append(rec.Obs, ObsRecord{X: z.X, Y: z.Y, Track: trackForObs[i]})
	}
	for _, t := range tr.tracks {
		st := "tentative"
		if t.state == Confirmed {
			st = "confirmed"
			if t.coasting {
				st = "coasting"
			}
		}
		rec.Tracks = append(rec.Tracks, TrackRecord{ID: t.ID, X: t.f.x[0], Y: t.f.x[1], State: st, Misses: t.misses})
	}
	return rec
}

// associate 构造带虚拟行/列的方阵代价矩阵并求全局最优匹配。
//
// 矩阵规模为 (nTracks+nObs)²：
//   - 左上 [航迹×观测]：可行对的平方马氏距离，超门限对填 bigCost（不可行）；
//   - 右上对角 [航迹×虚拟列]：航迹保持未关联的代价 = 门限 Gate2；
//   - 左下对角 [虚拟行×观测]：观测不关联任何航迹的代价 = 门限 Gate2；
//   - 右下 [虚拟×虚拟]：0。
//
// 这样任何代价优于门限的真实匹配都优于"不关联"，而超门限对永远不会被
// 强制关联——观测与航迹都允许在该帧保持未关联状态，且整体仍是最优二分图匹配。
func (tr *Tracker) associate(obs []Point) []int {
	nt, no := len(tr.tracks), len(obs)
	obsForTrack := make([]int, nt)
	for i := range obsForTrack {
		obsForTrack[i] = -1
	}
	if nt == 0 || no == 0 {
		return obsForTrack
	}
	n := nt + no
	cost := make([][]float64, n)
	for i := range cost {
		cost[i] = make([]float64, n)
		for j := range cost[i] {
			cost[i][j] = bigCost
		}
	}
	gate := tr.cfg.Gate2
	for ti, t := range tr.tracks {
		for oi, z := range obs {
			var d2 float64
			if t.coasting {
				// 滑行航迹使用冻结的门控协方差，搜索区域保持在丢失时的大小。
				d2 = mahalanobis2WithS(&t.f, z, t.coastS)
			} else {
				d2 = t.f.mahalanobis2(z, tr.cfg.MeasNoise)
			}
			if d2 <= gate {
				cost[ti][oi] = d2
			}
		}
		cost[ti][no+ti] = gate // 航迹未关联的代价
	}
	for oi := range obs {
		cost[nt+oi][oi] = gate // 观测未关联的代价
	}
	for i := 0; i < no; i++ {
		for j := 0; j < nt; j++ {
			cost[nt+i][no+j] = 0 // 虚拟-虚拟
		}
	}
	asg := hungarian(cost)
	for ti := 0; ti < nt; ti++ {
		if c := asg[ti]; c < no && cost[ti][c] < bigCost {
			obsForTrack[ti] = c
		}
	}
	return obsForTrack
}
