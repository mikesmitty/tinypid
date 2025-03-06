package tinypid

import "time"

// PIController implements a basic PI controller.
type PIController struct {
	// Config for the Controller.
	Config PIControllerConfig
	// State of the Controller.
	State PIControllerState
}

// PIControllerConfig contains configurable parameters for a Controller.
type PIControllerConfig struct {
	// ProportionalGain determines ratio of output response to error signal.
	ProportionalGain float32
	// IntegralGain determines previous error's affect on output.
	IntegralGain float32
	// MaxIntegralError is the maximum value of the integral error.
	MaxIntegralError float32
	// MinIntegralError is the minimum value of the integral error.
	MinIntegralError float32
	// MaxOutput is the max output from the PID.
	MaxOutput float32
	// MinOutput is the min output from the PID.
	MinOutput float32
}

// PIControllerState holds mutable state for a Controller.
type PIControllerState struct {
	// ControlErrorIntegral is the integrated control error over time.
	ControlErrorIntegral float32
	// ControlSignal is the current control signal output of the controller.
	ControlSignal float32
}

// PIControllerInput holds the input parameters to a Controller.
type PIControllerInput struct {
	// ReferenceSignal is the reference value for the signal to control.
	ReferenceSignal float32
	// ActualSignal is the actual value of the signal to control.
	ActualSignal float32
	// SamplingInterval is the time interval elapsed since the previous call of the controller Update method.
	SamplingInterval time.Duration
}

// Update the controller state.
func (c *PIController) Update(input PIControllerInput) {
	controlError := input.ReferenceSignal - input.ActualSignal
	c.State.ControlErrorIntegral = max(c.Config.MinIntegralError,
		min(c.Config.MaxIntegralError, c.State.ControlErrorIntegral+controlError*seconds(input.SamplingInterval)))
	c.State.ControlSignal = c.Config.ProportionalGain*controlError +
		c.Config.IntegralGain*c.State.ControlErrorIntegral
}

// Reset the controller state.
func (c *PIController) Reset() {
	c.State = PIControllerState{}
}
