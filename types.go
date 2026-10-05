package main

// Point 是二维平面上的一个观测点或位置。
type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// TrackState 表示航迹生命周期阶段。
type TrackState int

const (
	// Tentative 试探航迹：由未关联观测创建，需连续关联 ConfirmHits 帧才确认。
	Tentative TrackState = iota
	// Confirmed 确认航迹：稳定目标，允许连续 MaxMisses 帧漏检（遮挡滑行）。
	Confirmed
)

// Event 是某一帧发生的航迹生命周期事件，用于回放高亮。
type Event struct {
	Type    string  `json:"type"` // tentative / born / occluded / recovered / died / dropped
	TrackID int     `json:"track"`
	X       float64 `json:"x"`
	Y       float64 `json:"y"`
}

// ObsRecord 记录一帧中一个观测点及其关联结果。
type ObsRecord struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Track int     `json:"track"` // 关联到的航迹 ID，-1 表示未关联
}

// TrackRecord 记录一帧中一条存活航迹的快照。
type TrackRecord struct {
	ID     int     `json:"id"`
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	State  string  `json:"state"` // tentative / confirmed / coasting
	Misses int     `json:"misses"`
}

// FrameRecord 是一帧的完整回放记录。
type FrameRecord struct {
	Frame  int           `json:"frame"`
	Obs    []ObsRecord   `json:"obs"`
	Tracks []TrackRecord `json:"tracks"`
	Events []Event       `json:"events"`
}
