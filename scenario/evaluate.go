package scenario

import "github.com/TommyMarsss/multi-target-tracking/tracker"

// Metrics 是身份保持能力的评估指标。
type Metrics struct {
	Assignments          int     `json:"assignments"`          // 关联到真目标的观测总数
	CorrectAssignments   int     `json:"correctAssignments"`   // 与航迹多数派身份一致的关联数
	IdentityPreservation float64 `json:"identityPreservation"` // 身份保持正确率
	IDSwitchCount        int     `json:"idSwitchCount"`        // 航迹身份切换次数
}

// TrackIdentities 统计每条航迹关联到的真值目标分布：trackID -> trueID -> 次数。
func TrackIdentities(sc *Scene, frames []tracker.Frame) map[int]map[int]int {
	votes := map[int]map[int]int{}
	for fi, fr := range frames {
		for j, o := range fr.Obs {
			if o.AssignedTo < 0 {
				continue
			}
			tid := sc.Frames[fi][j].TrueID
			if tid < 0 {
				continue
			}
			if votes[o.AssignedTo] == nil {
				votes[o.AssignedTo] = map[int]int{}
			}
			votes[o.AssignedTo][tid]++
		}
	}
	return votes
}

// MajorityID 返回航迹的多数派真值身份及其纯度（多数票 / 总票）。
func MajorityID(votes map[int]int) (id int, purity float64) {
	total, best := 0, 0
	for tid, c := range votes {
		total += c
		if c > best {
			best, id = c, tid
		}
	}
	if total > 0 {
		purity = float64(best) / float64(total)
	}
	return id, purity
}

// Evaluate 对比真值与跟踪结果，计算身份保持指标。
// 关联到杂波（TrueID = -1）的观测不计入。
func Evaluate(sc *Scene, frames []tracker.Frame) Metrics {
	votes := TrackIdentities(sc, frames)
	lastTrue := map[int]int{}
	lastOk := map[int]bool{}
	var m Metrics
	for fi, fr := range frames {
		for j, o := range fr.Obs {
			if o.AssignedTo < 0 {
				continue
			}
			tid := sc.Frames[fi][j].TrueID
			if tid < 0 {
				continue
			}
			m.Assignments++
			if lastOk[o.AssignedTo] && lastTrue[o.AssignedTo] != tid {
				m.IDSwitchCount++
			}
			lastTrue[o.AssignedTo] = tid
			lastOk[o.AssignedTo] = true
		}
	}
	for _, v := range votes {
		_, best := 0, 0
		for _, c := range v {
			if c > best {
				best = c
			}
		}
		m.CorrectAssignments += best
	}
	if m.Assignments > 0 {
		m.IdentityPreservation = float64(m.CorrectAssignments) / float64(m.Assignments)
	}
	return m
}
