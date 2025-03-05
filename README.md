# TinyPID

[![PkgGoDev](https://pkg.go.dev/badge/github.com/mikesmitty/tinypid)](https://pkg.go.dev/github.com/mikesmitty/tinypid)
[![GoReportCard](https://goreportcard.com/badge/github.com/mikesmitty/tinypid)](https://goreportcard.com/report/github.com/mikesmitty/tinypid)

<p align="center">
  <img src="./doc/pid-go.svg" alt="logo"/>
</p>

`go.einride.tech/pid` PID controllers for Go, lightly ported from float64 to float32 for easier use with TinyGo

## Examples

### `pid.Controller`

A basic PID controller.

```go
import (
	"fmt"
	"time"

	pid "github.com/mikesmitty/tinypid"
)

func ExampleController() {
	// Create a PID controller.
	c := pid.Controller{
		Config: pid.ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
		},
	}
	// Update the PID controller.
	c.Update(pid.ControllerInput{
		ReferenceSignal:  10,
		ActualSignal:     0,
		SamplingInterval: 100 * time.Millisecond,
	})
	fmt.Printf("%+v\n", c.State)
	// Reset the PID controller.
	c.Reset()
	fmt.Printf("%+v\n", c.State)
	// Output:
	// {ControlError:10 ControlErrorIntegral:1 ControlErrorDerivative:100 ControlSignal:121}
	// {ControlError:0 ControlErrorIntegral:0 ControlErrorDerivative:0 ControlSignal:0}
}
```

*[Reference ≫](https://en.wikipedia.org/wiki/PID_controller)*

### `pid.AntiWindupController`

A PID-controller with low-pass filtering of the derivative term, feed forward
term, a saturated control output and anti-windup.

*[Reference ≫](http://www.cds.caltech.edu/~murray/amwiki)*

### `pid.TrackingController`

a PID-controller with low-pass filtering of the derivative term, feed forward
term, anti-windup and bumpless transfer using tracking mode control.

*[Reference ≫](http://www.cds.caltech.edu/~murray/amwiki)*
