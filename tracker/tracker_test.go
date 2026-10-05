package tracker

import (
	"math/rand"
	"testing"
)

// movingObs 生成一个匀速运动目标的观测序列（无噪声）。
func movingObs(x0, y0, vx, vy float64, frames int) [][]Point {
	out := make([][]Point, frames)
	for i := 0; i < frames; i++ {
		out[i] = []Point{{X: x0 + vx*float64(i), Y: y0 + vy*float64(i)}}
	}
	return out
}

func activeByID(tr *Tracker, id int) (Track, bool) {
	for _, t := range tr.Tracks() {
		if t.ID == id {
			return t, true
		}
	}
	return Track{}, false
}

// TestBirthRequiresConsecutiveHits 验证新航迹必须连续关联 ConfirmHits 帧才确认：
// 确认前为暂态，第 ConfirmHits 帧转为确认。
func TestBirthRequiresConsecutiveHits(t *testing.T) {
	cfg := DefaultConfig()
	tr := NewTracker(cfg)
	obs := movingObs(0, 100, 5, 0, 10)
	for i := 0; i < cfg.ConfirmHits+2; i++ {
		tr.Update(obs[i])
		tracks := tr.Tracks()
		if len(tracks) != 1 {
			t.Fatalf("第 %d 帧航迹数 = %d，期望 1", i, len(tracks))
		}
		want := Tentative
		if i+1 >= cfg.ConfirmHits {
			want = Confirmed
		}
		if tracks[0].State != want {
			t.Fatalf("第 %d 帧航迹状态 = %v，期望 %v", i, tracks[0].State, want)
		}
	}
}

// TestSingleFrameClutterNeverConfirms 验证每帧位置随机的杂波点
// 永远不会被确认为航迹，且暂态航迹会被快速剪除（抗航迹抖动）。
func TestSingleFrameClutterNeverConfirms(t *testing.T) {
	cfg := DefaultConfig()
	tr := NewTracker(cfg)
	rng := rand.New(rand.NewSource(7))
	for i := 0; i < 60; i++ {
		tr.Update([]Point{{X: rng.Float64() * 700, Y: rng.Float64() * 600}})
		for _, tk := range tr.Tracks() {
			if tk.State == Confirmed {
				t.Fatalf("第 %d 帧杂波被确认为航迹 #%d", i, tk.ID)
			}
		}
		// 暂态航迹最多存活 TentativeMaxMisses 帧，数量应有界
		if len(tr.Tracks()) > cfg.TentativeMaxMisses {
			t.Fatalf("第 %d 帧暂态航迹堆积：%d 条", i, len(tr.Tracks()))
		}
	}
}

// TestDeathRequiresConsecutiveMisses 验证确认航迹单帧漏检不消亡，
// 连续 MaxMisses 帧未关联才消亡。
func TestDeathRequiresConsecutiveMisses(t *testing.T) {
	cfg := DefaultConfig()
	tr := NewTracker(cfg)
	for _, obs := range movingObs(0, 100, 5, 0, 10) {
		tr.Update(obs)
	}
	tracks := tr.Tracks()
	if len(tracks) != 1 || tracks[0].State != Confirmed {
		t.Fatalf("前置条件失败：%v", tracks)
	}
	id := tracks[0].ID

	// MaxMisses-1 帧漏检：航迹必须仍然存活
	for i := 0; i < cfg.MaxMisses-1; i++ {
		tr.Update(nil)
		if _, ok := activeByID(tr, id); !ok {
			t.Fatalf("漏检 %d 帧后航迹被提前删除（阈值 %d）", i+1, cfg.MaxMisses)
		}
	}
	// 第 MaxMisses 帧漏检：航迹消亡
	frame := tr.Update(nil)
	if _, ok := activeByID(tr, id); ok {
		t.Fatalf("连续漏检 %d 帧后航迹未消亡", cfg.MaxMisses)
	}
	found := false
	for _, ts := range frame.Tracks {
		if ts.ID == id && ts.Event == "deleted" {
			found = true
		}
	}
	if !found {
		t.Fatalf("消亡帧未记录 deleted 事件：%+v", frame.Tracks)
	}
}

// TestOcclusionRecovery 验证短暂遮挡（漏检帧数 < MaxMisses）后，
// 目标重新出现时关联回原航迹，而不是产生新航迹。
func TestOcclusionRecovery(t *testing.T) {
	cfg := DefaultConfig()
	tr := NewTracker(cfg)
	// 目标匀速运动，先稳定跟踪 15 帧
	track := movingObs(0, 100, 5, 0, 40)
	for i := 0; i < 15; i++ {
		tr.Update(track[i])
	}
	tracks := tr.Tracks()
	if len(tracks) != 1 || tracks[0].State != Confirmed {
		t.Fatalf("前置条件失败：%v", tracks)
	}
	id := tracks[0].ID

	// 遮挡 3 帧（< MaxMisses）
	const occluded = 3
	for i := 0; i < occluded; i++ {
		tr.Update(nil)
	}
	if _, ok := activeByID(tr, id); !ok {
		t.Fatalf("遮挡期间航迹被删除")
	}

	// 遮挡结束，目标按原运动模型重现
	frame := tr.Update(track[15+occluded])

	// 1) 没有为此观测创建新航迹
	if len(tr.Tracks()) != 1 {
		t.Fatalf("遮挡恢复后航迹数 = %d，期望 1（不应新建航迹）", len(tr.Tracks()))
	}
	// 2) 观测关联回原航迹
	tk, _ := activeByID(tr, id)
	if tk.Misses != 0 {
		t.Fatalf("恢复后 Misses = %d，期望 0", tk.Misses)
	}
	// 3) 记录了 recovered 事件
	recovered := false
	for _, ts := range frame.Tracks {
		if ts.ID == id && ts.Event == "recovered" {
			recovered = true
		}
	}
	if !recovered {
		t.Fatalf("遮挡恢复帧未记录 recovered 事件：%+v", frame.Tracks)
	}
}

// TestInfeasibleMatchNotForced 验证远离所有航迹门限的观测不会被强行关联：
// 原航迹记一次漏检，观测另起暂态航迹，后续各归各。
func TestInfeasibleMatchNotForced(t *testing.T) {
	cfg := DefaultConfig()
	tr := NewTracker(cfg)
	for _, obs := range movingObs(100, 100, 5, 0, 10) {
		tr.Update(obs)
	}
	tracks := tr.Tracks()
	if len(tracks) != 1 || tracks[0].State != Confirmed {
		t.Fatalf("前置条件失败：%v", tracks)
	}
	id := tracks[0].ID

	// 一帧只有一个远处观测（远超关联门限）
	tr.Update([]Point{{X: 600, Y: 500}})
	tk, ok := activeByID(tr, id)
	if !ok {
		t.Fatalf("原航迹不应消亡")
	}
	if tk.Misses != 1 {
		t.Fatalf("原航迹 Misses = %d，期望 1（不可行匹配被拒绝）", tk.Misses)
	}
	if got := tk.Pos().X; got < 145 || got > 155 {
		t.Fatalf("原航迹位置被远处观测污染：x = %v", got)
	}
	// 远处观测应另起一条暂态航迹
	if len(tr.Tracks()) != 2 {
		t.Fatalf("航迹数 = %d，期望 2（远处观测另起暂态航迹）", len(tr.Tracks()))
	}

	// 下一帧原目标重现，应关联回原航迹
	tr.Update([]Point{{X: 155, Y: 100}})
	tk, _ = activeByID(tr, id)
	if tk.Misses != 0 {
		t.Fatalf("原航迹未恢复关联，Misses = %d", tk.Misses)
	}
}

// TestCrossingNoIdentitySwap 在解析构造的交叉场景下验证全局最优关联
// 不发生身份交换：两条航迹在交叉前后各自跟随同一目标。
func TestCrossingNoIdentitySwap(t *testing.T) {
	cfg := DefaultConfig()
	tr := NewTracker(cfg)
	// 目标 A 向右，目标 B 向左，第 20 帧附近在 x=200 处交叉（y 错开 6）
	const frames = 40
	var idA, idB int = -1, -1
	for i := 0; i < frames; i++ {
		ax := 100 + 5*float64(i)
		bx := 300 - 5*float64(i)
		tr.Update([]Point{{X: ax, Y: 297}, {X: bx, Y: 303}})
		if i == 0 {
			// 记录两条航迹与初始目标的对应关系
			for _, tk := range tr.Tracks() {
				if tk.Pos().X < 200 {
					idA = tk.ID
				} else {
					idB = tk.ID
				}
			}
		}
	}
	if idA < 0 || idB < 0 {
		t.Fatalf("未建立两条航迹")
	}
	// 交叉后：A 的航迹应在右侧继续向右，B 的航迹在左侧继续向左
	a, okA := activeByID(tr, idA)
	b, okB := activeByID(tr, idB)
	if !okA || !okB {
		t.Fatalf("交叉后航迹丢失：A=%v B=%v", okA, okB)
	}
	if a.Pos().X < 250 {
		t.Errorf("航迹 A 交叉后位置异常：x = %v（可能发生了身份交换）", a.Pos().X)
	}
	if b.Pos().X > 150 {
		t.Errorf("航迹 B 交叉后位置异常：x = %v（可能发生了身份交换）", b.Pos().X)
	}
	if a.Vel().X <= 0 || b.Vel().X >= 0 {
		t.Errorf("交叉后速度方向异常：A.vx=%v B.vx=%v", a.Vel().X, b.Vel().X)
	}
}
