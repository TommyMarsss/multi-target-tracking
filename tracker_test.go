package main

import (
	"math"
	"testing"
)

// runSeq 让跟踪器顺序处理一段观测序列，返回每帧记录与跟踪器。
func runSeq(cfg Config, seq [][]Point) ([]FrameRecord, *Tracker) {
	tr := NewTracker(cfg)
	recs := make([]FrameRecord, 0, len(seq))
	for _, obs := range seq {
		recs = append(recs, tr.Step(obs))
	}
	return recs, tr
}

func eventsOf(rec FrameRecord, typ string) []Event {
	var out []Event
	for _, ev := range rec.Events {
		if ev.Type == typ {
			out = append(out, ev)
		}
	}
	return out
}

func countEvents(recs []FrameRecord, typ string) int {
	n := 0
	for _, r := range recs {
		n += len(eventsOf(r, typ))
	}
	return n
}

// nearestTrack 返回该帧记录中距 p 最近的航迹 ID 及距离。
func nearestTrack(rec FrameRecord, p Point) (int, float64) {
	best, bd := -1, math.Inf(1)
	for _, tr := range rec.Tracks {
		d := math.Hypot(tr.X-p.X, tr.Y-p.Y)
		if d < bd {
			best, bd = tr.ID, d
		}
	}
	return best, bd
}

// 两条航迹交叉时，全局最优关联不得发生身份交换。
func TestCrossingNoIdentitySwap(t *testing.T) {
	cfg := DefaultConfig()
	posA := func(f int) Point { return Point{10 + 1.0*float64(f), 20 + 0.5*float64(f)} }
	posB := func(f int) Point { return Point{50 - 1.0*float64(f), 40 - 0.5*float64(f)} }
	// 两目标在第 20 帧于 (30,30) 处交汇，关联代价完全重叠。
	const frames = 41
	seq := make([][]Point, frames)
	for f := 0; f < frames; f++ {
		seq[f] = []Point{posA(f), posB(f)}
	}
	recs, _ := runSeq(cfg, seq)

	if n := countEvents(recs, "born"); n != 2 {
		t.Fatalf("应恰好新生 2 条航迹，实际 %d", n)
	}
	// 交叉前（第 5 帧，均已确认）记录两侧航迹 ID。
	idA, _ := nearestTrack(recs[5], posA(5))
	idB, _ := nearestTrack(recs[5], posB(5))
	if idA < 0 || idB < 0 || idA == idB {
		t.Fatalf("交叉前应存在两条不同航迹，得到 idA=%d idB=%d", idA, idB)
	}
	// 交叉后（末帧）身份必须保持：A 侧航迹仍是 idA，B 侧仍是 idB。
	gotA, dA := nearestTrack(recs[frames-1], posA(frames-1))
	gotB, dB := nearestTrack(recs[frames-1], posB(frames-1))
	if gotA != idA || gotB != idB {
		t.Fatalf("交叉后身份交换：A 侧=%d(应 %d)，B 侧=%d(应 %d)", gotA, idA, gotB, idB)
	}
	if dA > 3 || dB > 3 {
		t.Fatalf("末帧航迹偏离真值过大：dA=%.2f dB=%.2f", dA, dB)
	}
	if n := countEvents(recs, "died"); n != 0 {
		t.Fatalf("交叉场景不应有航迹消亡，实际 %d", n)
	}
}

// 新航迹必须连续 ConfirmHits 帧稳定出现才确认；孤立噪声点不得建立航迹。
func TestConfirmationThreshold(t *testing.T) {
	cfg := DefaultConfig() // ConfirmHits = 3
	seq := [][]Point{
		{{50, 35}, {90, 60}}, // 第 0 帧：真实目标 + 一个孤立噪声点
		{{50, 35}},
		{{50, 35}},
		{{50, 35}},
		{{50, 35}},
	}
	recs, _ := runSeq(cfg, seq)

	for f := 0; f < 2; f++ {
		for _, tr := range recs[f].Tracks {
			if tr.State != "tentative" {
				t.Fatalf("第 %d 帧不应存在确认航迹（确认阈值为 3 帧）", f)
			}
		}
	}
	if n := len(eventsOf(recs[2], "born")); n != 1 {
		t.Fatalf("真实目标应在第 3 帧（索引 2）确认，实际 born 事件 %d 个", n)
	}
	if n := countEvents(recs, "born"); n != 1 {
		t.Fatalf("全程应只有 1 条航迹确认（噪声点不得确认），实际 %d", n)
	}
	// 孤立噪声点：第 0 帧成为试探航迹，第 1 帧未再出现即被丢弃。
	if n := len(eventsOf(recs[0], "tentative")); n != 2 {
		t.Fatalf("第 0 帧应产生 2 条试探航迹，实际 %d", n)
	}
	if n := len(eventsOf(recs[1], "dropped")); n != 1 {
		t.Fatalf("孤立噪声点的试探航迹应在第 1 帧被丢弃，实际丢弃 %d 条", n)
	}
}

// 确认航迹必须连续 MaxMisses 帧未关联才消亡；单帧漏检不得删除航迹。
func TestDeletionThreshold(t *testing.T) {
	cfg := DefaultConfig()
	cfg.MaxMisses = 4

	// 目标第 0–9 帧在场，之后永久消失。
	seq := make([][]Point, 20)
	for f := 0; f < 10; f++ {
		seq[f] = []Point{{50, 35}}
	}
	recs, _ := runSeq(cfg, seq)

	// 第 10–13 帧漏检 1–4 次（未超阈值），航迹必须存活且处于滑行。
	for f := 10; f <= 13; f++ {
		if len(recs[f].Tracks) != 1 || recs[f].Tracks[0].State != "coasting" {
			t.Fatalf("第 %d 帧航迹应处于滑行状态，实际 %+v", f, recs[f].Tracks)
		}
	}
	// 第 14 帧漏检第 5 次（> MaxMisses=4），判定消亡。
	if n := len(eventsOf(recs[14], "died")); n != 1 {
		t.Fatalf("第 14 帧应判定航迹消亡，实际 died 事件 %d 个", n)
	}
	if len(recs[14].Tracks) != 0 {
		t.Fatalf("消亡后不应有存活航迹，实际 %d 条", len(recs[14].Tracks))
	}

	// 单帧漏检：目标第 10 帧缺失、第 11 帧恢复，航迹不得消亡。
	seq2 := make([][]Point, 16)
	for f := 0; f < 16; f++ {
		if f != 10 {
			seq2[f] = []Point{{50, 35}}
		}
	}
	recs2, _ := runSeq(cfg, seq2)
	if n := countEvents(recs2, "died"); n != 0 {
		t.Fatalf("单帧漏检不得导致消亡，实际 died 事件 %d 个", n)
	}
	if n := len(eventsOf(recs2[11], "recovered")); n != 1 {
		t.Fatalf("第 11 帧应恢复原航迹，实际 recovered 事件 %d 个", n)
	}
}

// 短暂遮挡（漏检帧数 ≤ MaxMisses）后必须重新关联回原航迹，而非建立新航迹。
func TestOcclusionRecovery(t *testing.T) {
	cfg := DefaultConfig() // MaxMisses = 5
	pos := func(f int) Point { return Point{20 + 0.8*float64(f), 20 + 0.3*float64(f)} }
	const frames = 41
	seq := make([][]Point, frames)
	for f := 0; f < frames; f++ {
		if f < 20 || f > 24 { // 第 20–24 帧遮挡（5 帧，恰在阈值内）
			seq[f] = []Point{pos(f)}
		}
	}
	recs, _ := runSeq(cfg, seq)

	if n := countEvents(recs, "born"); n != 1 {
		t.Fatalf("全程应只有 1 条航迹新生，实际 %d（遮挡后产生了新航迹）", n)
	}
	if n := countEvents(recs, "died"); n != 0 {
		t.Fatalf("短暂遮挡不得导致消亡，实际 died 事件 %d 个", n)
	}
	if n := len(eventsOf(recs[20], "occluded")); n != 1 {
		t.Fatalf("第 20 帧应标记遮挡，实际 occluded 事件 %d 个", n)
	}
	if n := len(eventsOf(recs[25], "recovered")); n != 1 {
		t.Fatalf("第 25 帧应恢复原航迹，实际 recovered 事件 %d 个", n)
	}
	// 遮挡前后航迹 ID 必须一致。
	idBefore, _ := nearestTrack(recs[19], pos(19))
	idAfter, _ := nearestTrack(recs[frames-1], pos(frames-1))
	if idBefore < 0 || idAfter != idBefore {
		t.Fatalf("遮挡恢复后航迹 ID 改变：遮挡前=%d，恢复后=%d", idBefore, idAfter)
	}
}

// 加噪观测 + 杂波 + 交叉 + 遮挡的完整场景下，身份保持正确率必须达标。
func TestIdentityPreservationUnderNoise(t *testing.T) {
	gts, frames := demoScenario()
	data, _ := RunScenario(gts, frames, DefaultConfig(), 1.0, 0.5, 42)
	s := data.Stats
	if s.IdentityAccuracy < 0.95 {
		t.Fatalf("身份保持正确率 %.2f%% 低于 95%%", s.IdentityAccuracy*100)
	}
	if s.IDSwitches != 0 {
		t.Fatalf("出现 %d 次 ID 切换", s.IDSwitches)
	}
	// 计划内的 5 帧遮挡必须恢复（偶发的单帧门控漏检也会产生恢复，属正常）。
	if s.Recovered < 1 {
		t.Fatalf("应至少发生 1 次遮挡恢复，实际 %d", s.Recovered)
	}
	// 4 个真值目标：T0/T1/T3 全程在场，T2 中途新生后消亡。
	if s.Born != 4 {
		t.Fatalf("应新生 4 条航迹，实际 %d", s.Born)
	}
	if s.Died != 1 {
		t.Fatalf("应消亡 1 条航迹（T2），实际 %d", s.Died)
	}
}
