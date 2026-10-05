package main

import (
	"math"
	"math/rand"
	"testing"
)

// bruteForce 用全排列枚举求方阵最小代价完美匹配，用于对照匈牙利算法的最优性。
func bruteForce(cost [][]float64) float64 {
	n := len(cost)
	perm := make([]int, n)
	for i := range perm {
		perm[i] = i
	}
	best := math.Inf(1)
	var rec func(k int)
	rec = func(k int) {
		if k == n {
			s := 0.0
			for i := 0; i < n; i++ {
				s += cost[i][perm[i]]
			}
			if s < best {
				best = s
			}
			return
		}
		for i := k; i < n; i++ {
			perm[k], perm[i] = perm[i], perm[k]
			rec(k + 1)
			perm[k], perm[i] = perm[i], perm[k]
		}
	}
	rec(0)
	return best
}

func hungarianCost(cost [][]float64) float64 {
	asg := hungarian(cost)
	s := 0.0
	for i, j := range asg {
		s += cost[i][j]
	}
	return s
}

// 随机方阵（含大惩罚值模拟不可行项）上，匈牙利算法必须达到暴力枚举的最优值。
func TestHungarianMatchesBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	for n := 1; n <= 7; n++ {
		for trial := 0; trial < 100; trial++ {
			cost := make([][]float64, n)
			for i := range cost {
				cost[i] = make([]float64, n)
				for j := range cost[i] {
					if rng.Float64() < 0.15 {
						cost[i][j] = bigCost // 不可行项
					} else {
						cost[i][j] = rng.Float64() * 10
					}
				}
			}
			got := hungarianCost(cost)
			want := bruteForce(cost)
			if math.Abs(got-want) > 1e-6*math.Max(1, math.Abs(want)) {
				t.Fatalf("n=%d trial=%d: hungarian=%v, bruteForce=%v", n, trial, got, want)
			}
		}
	}
}

// 带虚拟行/列的代价矩阵中，超门限的观测-航迹对不得被强制关联。
func TestHungarianRejectsInfeasiblePair(t *testing.T) {
	tr := NewTracker(DefaultConfig())
	// 一条已存在的航迹（直接注入，预测位置在 (10,10)）。
	tk := &Track{ID: 1, f: newKalman(Point{10, 10}, tr.cfg.MeasNoise), state: Confirmed, hits: 10}
	tr.tracks = []*Track{tk}
	tr.nextID = 2 // 避免与手工注入的航迹 ID 冲突
	tr.Step(nil)  // 推进一帧使其进入预测状态并记录漏检
	// 构造一个远离门限的观测：距离 ~50，马氏距离远超 Gate2。
	rec := tr.Step([]Point{{60, 60}})
	// 观测不得关联到航迹 1；它应成为一条新的试探航迹。
	if rec.Obs[0].Track == 1 {
		t.Fatalf("超门限观测被强制关联到航迹 1")
	}
	// 航迹 1 本帧应保持未关联（滑行），而不是被拉走。
	for _, trk := range rec.Tracks {
		if trk.ID == 1 && trk.State != "coasting" {
			t.Fatalf("航迹 1 应处于滑行状态，实际 %s", trk.State)
		}
	}
}
