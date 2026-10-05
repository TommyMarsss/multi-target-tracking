// Package scenario 生成带真值标签的多目标观测场景，用于测试与回放。
package scenario

import "math/rand"

// Point 是二维位置。
type Point struct{ X, Y float64 }

// Obs 是一个观测点，TrueID 为其来源目标的真值编号（-1 表示杂波）。
type Obs struct {
	X, Y   float64
	TrueID int
}

// Scene 是一段完整的观测序列。
type Scene struct {
	Width, Height float64
	Frames        [][]Obs
}

// Target 描述一个目标的运动学与存在区间。
type Target struct {
	ID         int
	Start, End int // 目标存在于 [Start, End) 帧
	P0, V      Point
	Occluded   func(t int) bool // 遮挡区间：目标存在但不产生观测
}

// Pos 返回目标在第 t 帧的真值位置；不存在或被遮挡时 ok = false。
func (tg Target) Pos(t int) (Point, bool) {
	if t < tg.Start || t >= tg.End {
		return Point{}, false
	}
	if tg.Occluded != nil && tg.Occluded(t) {
		return Point{}, false
	}
	return Point{tg.P0.X + tg.V.X*float64(t), tg.P0.Y + tg.V.Y*float64(t)}, true
}

// Config 是场景生成参数。
type Config struct {
	Seed       int64
	NumFrames  int
	MeasSigma  float64 // 量测噪声标准差
	ClutterMax int     // 每帧最大杂波点数（实际为 [0, ClutterMax] 均匀随机）
	Width      float64
	Height     float64
}

// Generate 按目标真值轨迹叠加量测噪声与杂波，生成观测序列。
func Generate(targets []Target, cfg Config) *Scene {
	rng := rand.New(rand.NewSource(cfg.Seed))
	sc := &Scene{Width: cfg.Width, Height: cfg.Height}
	for t := 0; t < cfg.NumFrames; t++ {
		var obs []Obs
		for _, tg := range targets {
			p, ok := tg.Pos(t)
			if !ok {
				continue
			}
			obs = append(obs, Obs{
				X:      p.X + rng.NormFloat64()*cfg.MeasSigma,
				Y:      p.Y + rng.NormFloat64()*cfg.MeasSigma,
				TrueID: tg.ID,
			})
		}
		nClutter := 0
		if cfg.ClutterMax > 0 {
			nClutter = rng.Intn(cfg.ClutterMax + 1)
		}
		for k := 0; k < nClutter; k++ {
			obs = append(obs, Obs{
				X:      rng.Float64() * cfg.Width,
				Y:      rng.Float64() * cfg.Height,
				TrueID: -1,
			})
		}
		rng.Shuffle(len(obs), func(i, j int) { obs[i], obs[j] = obs[j], obs[i] })
		sc.Frames = append(sc.Frames, obs)
	}
	return sc
}

// DemoScene 构建演示与集成测试共用的场景（120 帧，700x600）：
//   - 目标 1、2：对角交叉运动，在第 60 帧附近相距仅约 6px（约 3σ），
//     用于检验交叉歧义下的全局最优关联是否保持身份；
//   - 目标 3：第 20 帧新生，第 45–48 帧被遮挡（4 帧无观测），
//     第 90 帧起消失，用于检验新生确认、遮挡恢复与消亡判定；
//   - 每帧 0–2 个均匀分布的杂波点，用于检验抗噪声的新生逻辑。
func DemoScene(seed int64) *Scene {
	targets := []Target{
		{ID: 1, Start: 0, End: 120, P0: Point{100, 98}, V: Point{4, 2}},
		{ID: 2, Start: 0, End: 120, P0: Point{580, 104}, V: Point{-4, 2}},
		{ID: 3, Start: 20, End: 90, P0: Point{40, 520}, V: Point{3, -1},
			Occluded: func(t int) bool { return t >= 45 && t < 49 }},
	}
	return Generate(targets, Config{
		Seed: seed, NumFrames: 120, MeasSigma: 2, ClutterMax: 2,
		Width: 700, Height: 600,
	})
}
