package scenario

import (
	"testing"

	"github.com/TommyMarsss/multi-target-tracking/tracker"
)

// runScene 在场景上运行跟踪器，返回全部帧快照。
func runScene(sc *Scene, cfg tracker.Config) []tracker.Frame {
	tr := tracker.NewTracker(cfg)
	for _, obs := range sc.Frames {
		pts := make([]tracker.Point, len(obs))
		for i, o := range obs {
			pts[i] = tracker.Point{X: o.X, Y: o.Y}
		}
		tr.Update(pts)
	}
	return tr.Frames
}

// TestCrossingNoIdentitySwap 验证带噪声与杂波的完整场景中，
// 两条交叉目标各自由一条航迹稳定跟随，身份纯度 >= 90%，且不发生身份交换。
func TestCrossingNoIdentitySwap(t *testing.T) {
	sc := DemoScene(42)
	frames := runScene(sc, tracker.DefaultConfig())
	votes := TrackIdentities(sc, frames)

	// 找出跟随目标 1、2 的航迹（投票数最多的两条长航迹）
	trackOf := map[int]int{} // trueID -> trackID
	purityOf := map[int]float64{}
	for trackID, v := range votes {
		total := 0
		for _, c := range v {
			total += c
		}
		if total < 80 { // 只关注贯穿全程的长航迹
			continue
		}
		id, purity := MajorityID(v)
		if id != 1 && id != 2 {
			continue
		}
		if prev, dup := trackOf[id]; dup {
			t.Fatalf("目标 %d 被多条长航迹（#%d、#%d）跟随，可能发生身份分裂", id, prev, trackID)
		}
		trackOf[id] = trackID
		purityOf[id] = purity
	}
	for _, id := range []int{1, 2} {
		if _, ok := trackOf[id]; !ok {
			t.Fatalf("目标 %d 没有对应的长航迹", id)
		}
		if purityOf[id] < 0.9 {
			t.Errorf("目标 %d 的航迹身份纯度 = %.2f，期望 >= 0.9", id, purityOf[id])
		}
	}
	if trackOf[1] == trackOf[2] {
		t.Fatalf("目标 1、2 共用航迹 #%d，发生身份交换", trackOf[1])
	}

	m := Evaluate(sc, frames)
	if m.IDSwitchCount != 0 {
		t.Errorf("身份切换次数 = %d，期望 0", m.IDSwitchCount)
	}
}

// TestIdentityPreservationUnderNoise 验证噪声观测下的整体身份保持正确率。
func TestIdentityPreservationUnderNoise(t *testing.T) {
	sc := DemoScene(42)
	frames := runScene(sc, tracker.DefaultConfig())
	m := Evaluate(sc, frames)
	t.Logf("关联 %d 次，身份保持率 %.3f，ID 切换 %d 次",
		m.Assignments, m.IdentityPreservation, m.IDSwitchCount)
	if m.IdentityPreservation < 0.9 {
		t.Errorf("身份保持率 = %.3f，期望 >= 0.9", m.IdentityPreservation)
	}
}

// TestOcclusionRecoveryInScene 验证场景中目标 3 遮挡 4 帧后
// 仍由同一条航迹恢复关联，而不是新建航迹。
func TestOcclusionRecoveryInScene(t *testing.T) {
	sc := DemoScene(42)
	frames := runScene(sc, tracker.DefaultConfig())
	votes := TrackIdentities(sc, frames)

	// 目标 3 应只对应一条航迹
	var owners []int
	for trackID, v := range votes {
		if id, _ := MajorityID(v); id == 3 {
			owners = append(owners, trackID)
		}
	}
	if len(owners) != 1 {
		t.Fatalf("目标 3 对应 %d 条航迹，期望 1（遮挡恢复不应产生新航迹）", len(owners))
	}
	owner := owners[0]

	// 遮挡结束帧（49）附近应出现 recovered 事件
	recovered := false
	for _, fr := range frames {
		for _, ts := range fr.Tracks {
			if ts.ID == owner && ts.Event == "recovered" {
				recovered = true
				if fr.Index < 49 || fr.Index > 52 {
					t.Errorf("recovered 事件出现在第 %d 帧，期望 49–52", fr.Index)
				}
			}
		}
	}
	if !recovered {
		t.Errorf("航迹 #%d 未记录 recovered 事件", owner)
	}
}

// TestBirthAndDeathThresholds 验证目标 3 的新生确认与消亡判定
// 符合连续帧阈值：第 20 帧出现，ConfirmHits 帧后确认；
// 第 90 帧消失，MaxMisses 帧后消亡。
func TestBirthAndDeathThresholds(t *testing.T) {
	sc := DemoScene(42)
	cfg := tracker.DefaultConfig()
	frames := runScene(sc, cfg)
	votes := TrackIdentities(sc, frames)

	var owner = -1
	for trackID, v := range votes {
		if id, _ := MajorityID(v); id == 3 {
			owner = trackID
		}
	}
	if owner < 0 {
		t.Fatalf("目标 3 没有对应航迹")
	}

	bornFrame, confirmedFrame, deletedFrame := -1, -1, -1
	for _, fr := range frames {
		for _, ts := range fr.Tracks {
			if ts.ID != owner {
				continue
			}
			if bornFrame < 0 {
				bornFrame = fr.Index
			}
			if ts.Event == "confirmed" {
				confirmedFrame = fr.Index
			}
			if ts.Event == "deleted" {
				deletedFrame = fr.Index
			}
		}
	}
	// 目标第 20 帧首次产生观测
	if bornFrame != 20 {
		t.Errorf("新生帧 = %d，期望 20", bornFrame)
	}
	// 连续关联 ConfirmHits 帧后确认
	if confirmedFrame != 20+cfg.ConfirmHits-1 {
		t.Errorf("确认帧 = %d，期望 %d", confirmedFrame, 20+cfg.ConfirmHits-1)
	}
	// 目标第 90 帧起消失，连续 MaxMisses 帧漏检后消亡
	wantDeleted := 90 + cfg.MaxMisses - 1
	if deletedFrame != wantDeleted {
		t.Errorf("消亡帧 = %d，期望 %d", deletedFrame, wantDeleted)
	}
}
