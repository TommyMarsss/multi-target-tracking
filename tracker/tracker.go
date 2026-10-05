package tracker

import "sort"

// TrackState 是航迹的生命周期状态。
type TrackState int

const (
	// Tentative 暂态航迹：由未关联观测创建，需连续关联 ConfirmHits 帧才确认。
	Tentative TrackState = iota
	// Confirmed 确认航迹：稳定关联后的正式航迹。
	Confirmed
)

func (s TrackState) String() string {
	if s == Confirmed {
		return "confirmed"
	}
	return "tentative"
}

// Track 表示一条航迹。
type Track struct {
	ID     int
	State  TrackState
	Hits   int // 累计关联次数
	Consec int // 连续关联帧数（用于确认判定）
	Misses int // 连续未关联帧数（用于消亡判定）
	Age    int // 存活帧数
	kf     kfState
}

// Pos 返回航迹当前的估计位置。
func (t Track) Pos() Point { return Point{X: t.kf.x[0], Y: t.kf.x[1]} }

// Vel 返回航迹当前的估计速度。
func (t Track) Vel() Point { return Point{X: t.kf.x[2], Y: t.kf.x[3]} }

// Config 是跟踪器的全部可调参数。
type Config struct {
	ProcessNoise       float64 // 过程噪声强度 q（加速度谱密度）
	MeasNoise          float64 // 量测噪声方差 r
	InitPosVar         float64 // 新航迹初始位置方差
	InitVelVar         float64 // 新航迹初始速度方差（大值表示速度未知）
	Gate               float64 // 关联门限（马氏距离平方，2D 卡方 99% 约为 9.21）
	ConfirmHits        int     // 暂态航迹连续关联多少帧后确认
	TentativeMaxMisses int     // 暂态航迹连续漏关联多少帧后剪除
	MaxMisses          int     // 确认航迹连续漏关联多少帧后消亡
}

// DefaultConfig 返回一组对演示场景调校好的默认参数。
func DefaultConfig() Config {
	return Config{
		ProcessNoise:       0.5,
		MeasNoise:          4, // σ = 2
		InitPosVar:         4,
		InitVelVar:         100,
		Gate:               9.21,
		ConfirmHits:        3,
		TentativeMaxMisses: 2,
		MaxMisses:          5,
	}
}

// ObsSnapshot 是一帧中一个观测点的回放快照。
type ObsSnapshot struct {
	X          float64 `json:"x"`
	Y          float64 `json:"y"`
	AssignedTo int     `json:"assignedTo"` // 关联到的航迹 ID，-1 表示未关联
}

// TrackSnapshot 是一帧中一条航迹的回放快照。
type TrackSnapshot struct {
	ID     int     `json:"id"`
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	State  string  `json:"state"`
	Misses int     `json:"misses"`
	Event  string  `json:"event"` // "", "born", "confirmed", "recovered", "deleted", "pruned"
}

// Frame 是一帧处理结果的完整快照，用于回放与评估。
type Frame struct {
	Index  int             `json:"index"`
	Obs    []ObsSnapshot   `json:"obs"`
	Tracks []TrackSnapshot `json:"tracks"`
}

// Tracker 维护一组航迹，逐帧处理观测集合。
type Tracker struct {
	cfg    Config
	tracks []*Track
	nextID int
	Frames []Frame // 每一帧的快照，供回放与离线评估
}

// NewTracker 创建跟踪器。
func NewTracker(cfg Config) *Tracker {
	return &Tracker{cfg: cfg}
}

// Tracks 返回当前存活航迹的副本（含暂态航迹）。
func (t *Tracker) Tracks() []Track {
	out := make([]Track, len(t.tracks))
	for i, tr := range t.tracks {
		out[i] = *tr
	}
	return out
}

// Update 处理一帧观测，返回该帧快照。
//
// 处理流程：预测 → 构建门控代价矩阵 → 全局最优关联 → 状态更新 →
// 生命周期管理（消亡判定）→ 新航迹创建 → 记录快照。
func (t *Tracker) Update(obs []Point) Frame {
	cfg := t.cfg

	// 1. 预测所有存活航迹
	for _, tr := range t.tracks {
		tr.kf.predict(cfg.ProcessNoise)
		tr.Age++
	}

	// 2. 构建门控代价矩阵（马氏距离平方，超门限记为不可行）
	nT, nO := len(t.tracks), len(obs)
	costs := make([][]float64, nT)
	for i, tr := range t.tracks {
		costs[i] = make([]float64, nO)
		for j := range obs {
			y, s := tr.kf.innovation(obs[j], cfg.MeasNoise)
			d2 := mahalanobis(y, inv2(s))
			if d2 >= cfg.Gate {
				costs[i][j] = bigCost
			} else {
				costs[i][j] = d2
			}
		}
	}

	// 3. 两阶段全局最优关联（匈牙利算法 + 显式未关联建模）。
	// 第一阶段确认航迹关联全部观测；第二阶段暂态航迹只关联剩余观测。
	// 新生暂态航迹协方差涣散、门限范围大，若与确认航迹同场竞价，
	// 容易凭较小的马氏距离抢走确认航迹的观测（身份劫持），两阶段可杜绝。
	asg := assignment{
		trackToObs: make([]int, nT),
		obsToTrack: make([]int, nO),
	}
	for i := range asg.trackToObs {
		asg.trackToObs[i] = -1
	}
	for j := range asg.obsToTrack {
		asg.obsToTrack[j] = -1
	}
	var confirmed, tentative []int
	for i, tr := range t.tracks {
		if tr.State == Confirmed {
			confirmed = append(confirmed, i)
		} else {
			tentative = append(tentative, i)
		}
	}
	allObs := make([]int, nO)
	for j := range allObs {
		allObs[j] = j
	}
	stageAssign := func(trackIdx, obsIdx []int) {
		if len(trackIdx) == 0 || len(obsIdx) == 0 {
			return
		}
		sub := make([][]float64, len(trackIdx))
		for a, i := range trackIdx {
			sub[a] = make([]float64, len(obsIdx))
			for b, j := range obsIdx {
				sub[a][b] = costs[i][j]
			}
		}
		a := globalAssignment(sub, len(trackIdx), len(obsIdx), cfg.Gate)
		for ai, bj := range a.trackToObs {
			if bj >= 0 {
				asg.trackToObs[trackIdx[ai]] = obsIdx[bj]
				asg.obsToTrack[obsIdx[bj]] = trackIdx[ai]
			}
		}
	}
	stageAssign(confirmed, allObs)
	var freeObs []int
	for j := range obs {
		if asg.obsToTrack[j] < 0 {
			freeObs = append(freeObs, j)
		}
	}
	stageAssign(tentative, freeObs)

	frame := Frame{Index: len(t.Frames)}
	frame.Obs = make([]ObsSnapshot, nO)
	for j := range obs {
		frame.Obs[j] = ObsSnapshot{X: obs[j].X, Y: obs[j].Y, AssignedTo: -1}
	}

	// 4. 应用关联结果
	events := map[int]string{}
	for i, tr := range t.tracks {
		j := asg.trackToObs[i]
		if j < 0 {
			tr.Misses++
			tr.Consec = 0
			continue
		}
		// 确认航迹漏关联后重新关联 → 遮挡恢复
		recovered := tr.State == Confirmed && tr.Misses > 0
		tr.kf.update(obs[j], cfg.MeasNoise)
		tr.Hits++
		tr.Consec++
		tr.Misses = 0
		frame.Obs[j].AssignedTo = tr.ID
		switch {
		case recovered:
			events[tr.ID] = "recovered"
		case tr.State == Tentative && tr.Consec >= cfg.ConfirmHits:
			tr.State = Confirmed
			events[tr.ID] = "confirmed"
		}
	}

	// 5. 消亡判定：连续漏关联达到阈值才删除，单帧漏检不致命
	var deleted []*Track
	kept := t.tracks[:0]
	for _, tr := range t.tracks {
		limit := cfg.MaxMisses
		if tr.State == Tentative {
			limit = cfg.TentativeMaxMisses
		}
		if tr.Misses >= limit {
			deleted = append(deleted, tr)
		} else {
			kept = append(kept, tr)
		}
	}
	t.tracks = kept

	// 6. 未关联观测 → 创建暂态航迹（需连续关联确认，抗噪声抖动）
	for j := range obs {
		if asg.obsToTrack[j] >= 0 {
			continue
		}
		tr := &Track{ID: t.nextID, State: Tentative, Hits: 1, Consec: 1}
		t.nextID++
		tr.kf.x = [4]float64{obs[j].X, obs[j].Y, 0, 0}
		tr.kf.p = [4][4]float64{
			{cfg.InitPosVar, 0, 0, 0},
			{0, cfg.InitPosVar, 0, 0},
			{0, 0, cfg.InitVelVar, 0},
			{0, 0, 0, cfg.InitVelVar},
		}
		t.tracks = append(t.tracks, tr)
		events[tr.ID] = "born"
	}

	// 7. 记录快照
	for _, tr := range t.tracks {
		frame.Tracks = append(frame.Tracks, TrackSnapshot{
			ID: tr.ID, X: tr.kf.x[0], Y: tr.kf.x[1],
			State: tr.State.String(), Misses: tr.Misses, Event: events[tr.ID],
		})
	}
	for _, tr := range deleted {
		ev := "deleted"
		if tr.State == Tentative {
			ev = "pruned"
		}
		frame.Tracks = append(frame.Tracks, TrackSnapshot{
			ID: tr.ID, X: tr.kf.x[0], Y: tr.kf.x[1],
			State: tr.State.String(), Misses: tr.Misses, Event: ev,
		})
	}
	sort.Slice(frame.Tracks, func(a, b int) bool { return frame.Tracks[a].ID < frame.Tracks[b].ID })

	t.Frames = append(t.Frames, frame)
	return frame
}
