package main

import (
	"flag"
	"fmt"
	"os"
)

// 运行演示场景（交叉 + 中途新生/消亡 + 遮挡恢复 + 随机杂波），
// 生成单文件 HTML 回放报告。
func main() {
	out := flag.String("o", "report.html", "输出 HTML 文件路径")
	seed := flag.Int64("seed", 42, "随机种子")
	flag.Parse()

	gts, frames := demoScenario()
	cfg := DefaultConfig()
	data, _ := RunScenario(gts, frames, cfg, 1.0, 0.5, *seed)

	if err := WriteHTML(*out, data); err != nil {
		fmt.Fprintln(os.Stderr, "生成报告失败:", err)
		os.Exit(1)
	}
	s := data.Stats
	fmt.Printf("仿真完成：%d 帧 | 新生 %d 消亡 %d 遮挡 %d 恢复 %d | 身份保持率 %.1f%% | ID 切换 %d 次\n",
		s.Frames, s.Born, s.Died, s.Occlusions, s.Recovered, s.IdentityAccuracy*100, s.IDSwitches)
	fmt.Println("报告已生成:", *out)
}
