package main

import "math"

// hungarian 求解 n×n 方阵的最小代价完美匹配（匈牙利算法，O(n³)，
// 基于势的 O(n³) 实现）。返回 assign，其中 assign[i] 为第 i 行匹配到的列号。
// 代价矩阵中允许出现大的有限惩罚值（如 bigCost），但不允许 ±Inf/NaN。
func hungarian(cost [][]float64) []int {
	n := len(cost)
	// 1-indexed：u、v 为行/列势，p[j] 为列 j 匹配到的行，way 用于增广路回溯。
	u := make([]float64, n+1)
	v := make([]float64, n+1)
	p := make([]int, n+1)
	way := make([]int, n+1)
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
		// 沿增广路翻转匹配。
		for {
			j1 := way[j0]
			p[j0] = p[j1]
			j0 = j1
			if j0 == 0 {
				break
			}
		}
	}
	assign := make([]int, n)
	for j := 1; j <= n; j++ {
		if p[j] > 0 {
			assign[p[j]-1] = j - 1
		}
	}
	return assign
}
