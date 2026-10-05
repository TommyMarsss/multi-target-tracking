package tracker

import "math"

// bigCost 表示代价矩阵中的不可行（被门限拒绝的）匹配。
// 取值远大于任何可行代价与未匹配代价之和，保证匈牙利算法不会选择它。
const bigCost = 1e12

// hungarian 求解 n×n 方阵的最小代价分配问题（匈牙利算法，O(n^3)）。
// 返回 match，match[i] = j 表示第 i 行分配给第 j 列。
// 要求所有元素为有限值（不可行项用 bigCost 表示）。
func hungarian(cost [][]float64) []int {
	n := len(cost)
	u := make([]float64, n+1) // 行势
	v := make([]float64, n+1) // 列势
	p := make([]int, n+1)     // p[j] = 与列 j 匹配的行
	way := make([]int, n+1)   // 增广路回溯
	for i := 1; i <= n; i++ {
		p[0] = i
		j0 := 0
		minv := make([]float64, n+1)
		used := make([]bool, n+1)
		for j := range minv {
			minv[j] = math.Inf(1)
		}
		for {
			used[j0] = true
			i0 := p[j0]
			delta := math.Inf(1)
			j1 := 0
			for j := 1; j <= n; j++ {
				if used[j] {
					continue
				}
				cur := cost[i0-1][j-1] - u[i0] - v[j]
				if cur < minv[j] {
					minv[j] = cur
					way[j] = j0
				}
				if minv[j] < delta {
					delta = minv[j]
					j1 = j
				}
			}
			for j := 0; j <= n; j++ {
				if used[j] {
					u[p[j]] += delta
					v[j] -= delta
				} else {
					minv[j] -= delta
				}
			}
			j0 = j1
			if p[j0] == 0 {
				break
			}
		}
		// 沿增广路翻转匹配
		for {
			j1 := way[j0]
			p[j0] = p[j1]
			j0 = j1
			if j0 == 0 {
				break
			}
		}
	}
	match := make([]int, n)
	for i := range match {
		match[i] = -1
	}
	for j := 1; j <= n; j++ {
		if p[j] >= 1 {
			match[p[j]-1] = j - 1
		}
	}
	return match
}

// assignment 保存一次全局最优关联的结果。
type assignment struct {
	trackToObs []int // trackToObs[i] = j 表示航迹 i 关联观测 j，-1 表示未关联
	obsToTrack []int // obsToTrack[j] = i 表示观测 j 关联航迹 i，-1 表示未关联
}

// globalAssignment 在带门限的代价矩阵上求全局最优观测-航迹分配。
//
// 做法：把 nTracks + nObs 阶方阵补全——
//   - 左上块：航迹-观测的（已门控）代价；
//   - 右上对角：航迹保持未关联的代价 gate；
//   - 左下对角：观测保持未关联的代价 gate；
//   - 右下块：虚拟-虚拟，代价 0。
//
// 这样"不关联"被显式建模为一种代价为 gate 的选择，匈牙利算法求出的
// 全局最优解中，任何代价 >= gate 的航迹-观测对都不会被强制关联，
// 而是各自落入未关联状态。costs 中 >= gate 或 bigCost 的项视为不可行。
func globalAssignment(costs [][]float64, nTracks, nObs int, gate float64) assignment {
	res := assignment{
		trackToObs: make([]int, nTracks),
		obsToTrack: make([]int, nObs),
	}
	for i := range res.trackToObs {
		res.trackToObs[i] = -1
	}
	for j := range res.obsToTrack {
		res.obsToTrack[j] = -1
	}
	if nTracks == 0 || nObs == 0 {
		return res
	}

	n := nTracks + nObs
	c := make([][]float64, n)
	for i := range c {
		c[i] = make([]float64, n)
		for j := range c[i] {
			c[i][j] = bigCost
		}
	}
	for t := 0; t < nTracks; t++ {
		for o := 0; o < nObs; o++ {
			c[t][o] = costs[t][o]
		}
		c[t][nObs+t] = gate // 航迹 t 未关联的代价
	}
	for o := 0; o < nObs; o++ {
		c[nTracks+o][o] = gate // 观测 o 未关联的代价
		for t := 0; t < nTracks; t++ {
			c[nTracks+o][nObs+t] = 0 // 虚拟-虚拟
		}
	}

	m := hungarian(c)
	for t := 0; t < nTracks; t++ {
		j := m[t]
		// 只接受代价严格小于门限的可行匹配；其余保持未关联。
		if j >= 0 && j < nObs && costs[t][j] < gate {
			res.trackToObs[t] = j
			res.obsToTrack[j] = t
		}
	}
	return res
}
