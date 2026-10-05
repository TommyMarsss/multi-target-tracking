// replay 在演示场景上运行跟踪器，把逐帧结果与评估指标内嵌进一个
// 单一的静态 HTML 文件（replay.html），浏览器打开即可逐帧回放。
package main

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"os"
	"strings"

	"github.com/TommyMarsss/multi-target-tracking/scenario"
	"github.com/TommyMarsss/multi-target-tracking/tracker"
)

//go:embed template.html
var templateHTML string

type payload struct {
	Width   float64          `json:"width"`
	Height  float64          `json:"height"`
	Frames  []tracker.Frame  `json:"frames"`
	Metrics scenario.Metrics `json:"metrics"`
	Config  tracker.Config   `json:"config"`
}

func main() {
	sc := scenario.DemoScene(42)
	cfg := tracker.DefaultConfig()
	tr := tracker.NewTracker(cfg)
	for _, obs := range sc.Frames {
		pts := make([]tracker.Point, len(obs))
		for i, o := range obs {
			pts[i] = tracker.Point{X: o.X, Y: o.Y}
		}
		tr.Update(pts)
	}
	metrics := scenario.Evaluate(sc, tr.Frames)

	data, err := json.Marshal(payload{
		Width: sc.Width, Height: sc.Height,
		Frames: tr.Frames, Metrics: metrics, Config: cfg,
	})
	if err != nil {
		fmt.Fprintln(os.Stderr, "序列化失败:", err)
		os.Exit(1)
	}
	html := strings.Replace(templateHTML, "__DATA__", string(data), 1)
	if err := os.WriteFile("replay.html", []byte(html), 0o644); err != nil {
		fmt.Fprintln(os.Stderr, "写入 replay.html 失败:", err)
		os.Exit(1)
	}
	fmt.Printf("已生成 replay.html（%d 帧）\n", len(tr.Frames))
	fmt.Printf("关联 %d 次，身份保持率 %.1f%%，ID 切换 %d 次\n",
		metrics.Assignments, metrics.IdentityPreservation*100, metrics.IDSwitchCount)
}
