package tinypid

import (
	"math"
	"time"
)

// Taken from math.MaxFloat32
const MaxFloat32 = 0x1p127 * (1 + (1 - 0x1p-23))

func Abs(x float32) float32 {
	return math.Float32frombits(math.Float32bits(x) &^ (1 << 31))
}

func IsNaN(f float32) bool {
	return f != f
}

// Convert time.Duration (int64 nanoseconds) to float32 seconds
func seconds(d time.Duration) float32 {
	return float32(d) / 1e9
}
