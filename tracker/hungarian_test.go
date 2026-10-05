package tracker

import "testing"

// bruteForce 用全排列求最小代价，作为匈牙利算法的对照真值。
func bruteForce(cost [][]float64) float64 {
	n := len(cost)
	perm := make([]int, n)
	for i := range perm {
		perm[i] = i
	}
	best := -1.0
	var visit func(k int)
	visit = func(k int) {
		if k == n {
			total := 0.0
			for i := 0; i < n; i++ {
				total += cost[i][perm[i]]
			}
			if best < 0 || total < best {
				best = total
			}
			return
		}
		for i := k; i < n; i++ {
			perm[k], perm[i] = perm[i], perm[k]
			visit(k + 1)
			perm[k], perm[i] = perm[i], perm[k]
		}
	}
	visit(0)
	return best
}

func totalCost(cost [][]float64, match []int) float64 {
	total := 0.0
	for i, j := range match {
		total += cost[i][j]
	}
	return total
}

func TestHungarianKnownOptimum(t *testing.T) {
	cost := [][]float64{
		{4, 1, 3},
		{2, 0, 5},
		{3, 2, 2},
	}
	m := hungarian(cost)
	// 最优：0->1(1), 1->0(2), 2->2(2)，总代价 5
	if got := totalCost(cost, m); got != 5 {
		t.Fatalf("总代价 = %v，期望 5（匹配 %v）", got, m)
	}
}

// TestHungarianBeatsGreedy 验证贪心最近邻会失败、全局最优能胜出的矩阵：
// 贪心按行取最近邻得 1 + 100 = 101，全局最优为 1.5 + 1.4 = 2.9。
func TestHungarianBeatsGreedy(t *testing.T) {
	cost := [][]float64{
		{1, 1.5},
		{1.4, 100},
	}
	m := hungarian(cost)
	if m[0] != 1 || m[1] != 0 {
		t.Fatalf("匹配 = %v，期望 [1 0]", m)
	}
	if got := totalCost(cost, m); got != 2.9 {
		t.Fatalf("总代价 = %v，期望 2.9", got)
	}
}

func TestHungarianMatchesBruteForce(t *testing.T) {
	cases := [][][]float64{
		{{4, 1, 3}, {2, 0, 5}, {3, 2, 2}},
		{{9, 2, 7, 8}, {6, 4, 3, 7}, {5, 8, 1, 8}, {7, 6, 9, 4}},
		{{0.5, 0.3, 0.2}, {0.1, 0.4, 0.6}, {0.9, 0.8, 0.7}},
		{{bigCost, 1, 2}, {1, bigCost, 3}, {2, 3, bigCost}},
	}
	for i, cost := range cases {
		m := hungarian(cost)
		got, want := totalCost(cost, m), bruteForce(cost)
		if got != want {
			t.Errorf("用例 %d：匈牙利总代价 %v != 暴力最优 %v", i, got, want)
		}
	}
}

// TestGlobalAssignmentRejectsInfeasible 验证超门限的观测-航迹对不会被强制关联，
// 航迹与观测都保持未关联状态。
func TestGlobalAssignmentRejectsInfeasible(t *testing.T) {
	costs := [][]float64{{bigCost}}
	a := globalAssignment(costs, 1, 1, 9.21)
	if a.trackToObs[0] != -1 {
		t.Errorf("不可行匹配被强制关联：trackToObs = %v", a.trackToObs)
	}
	if a.obsToTrack[0] != -1 {
		t.Errorf("不可行观测被强制关联：obsToTrack = %v", a.obsToTrack)
	}
}

// TestGlobalAssignmentPrefersFeasible 验证可行匹配优先于未关联，
// 且全局最优解避开不可行项。
func TestGlobalAssignmentPrefersFeasible(t *testing.T) {
	const gate = 9.21
	costs := [][]float64{
		{1, bigCost},
		{bigCost, 2},
	}
	a := globalAssignment(costs, 2, 2, gate)
	if a.trackToObs[0] != 0 || a.trackToObs[1] != 1 {
		t.Errorf("trackToObs = %v，期望 [0 1]", a.trackToObs)
	}
}

// TestGlobalAssignmentUnmatchedWhenCheaper 验证当关联代价高于门限时，
// 算法选择让双方保持未关联（代价各为 gate），而不是强行配对。
func TestGlobalAssignmentUnmatchedWhenCheaper(t *testing.T) {
	const gate = 10.0
	// 唯一可能的配对代价 15 > gate，应保持双方未关联。
	costs := [][]float64{{15}}
	a := globalAssignment(costs, 1, 1, gate)
	if a.trackToObs[0] != -1 || a.obsToTrack[0] != -1 {
		t.Errorf("超门限配对被关联：%v / %v", a.trackToObs, a.obsToTrack)
	}
}

func TestGlobalAssignmentEmpty(t *testing.T) {
	a := globalAssignment(nil, 0, 3, 9.21)
	if len(a.obsToTrack) != 3 {
		t.Errorf("obsToTrack 长度 = %d，期望 3", len(a.obsToTrack))
	}
	for _, v := range a.obsToTrack {
		if v != -1 {
			t.Errorf("无航迹时观测不应被关联：%v", a.obsToTrack)
		}
	}
	a = globalAssignment(nil, 2, 0, 9.21)
	if len(a.trackToObs) != 2 {
		t.Errorf("trackToObs 长度 = %d，期望 2", len(a.trackToObs))
	}
}
