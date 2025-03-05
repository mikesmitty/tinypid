package tinypid

import (
	"time"
)

// TrackingController implements a PID-controller with low-pass filter of the derivative term,
// feed forward term, anti-windup and bumpless transfer using tracking mode control.
//
// The anti-windup and bumpless transfer mechanisms use a tracking mode as defined in
// Chapter 6 of Åström and Murray, Feedback Systems:
// An Introduction to Scientists and Engineers, 2008
// (http://www.cds.caltech.edu/~murray/amwiki)
//
// The ControlError, ControlErrorIntegrand, ControlErrorIntegral and ControlErrorDerivative are prevented
// from reaching +/- inf by clamping them to [-MaxFloat32, MaxFloat32].
type TrackingController struct {
	// Config for the TrackingController.
	Config TrackingControllerConfig
	// State of the TrackingController.
	State TrackingControllerState
}

// TrackingControllerConfig contains configurable parameters for a TrackingController.
type TrackingControllerConfig struct {
	// ProportionalGain is the P part gain.
	ProportionalGain float32
	// IntegralGain is the I part gain.
	IntegralGain float32
	// DerivativeGain is the D part gain.
	DerivativeGain float32
	// AntiWindUpGain is the anti-windup tracking gain.
	AntiWindUpGain float32
	// IntegralDischargeTimeConstant is the time constant to discharge the integral state of the PID controller (s)
	IntegralDischargeTimeConstant float32
	// LowPassTimeConstant is the D part low-pass filter time constant => cut-off frequency 1/LowPassTimeConstant.
	LowPassTimeConstant time.Duration
	// MaxOutput is the max output from the PID.
	MaxOutput float32
	// MinOutput is the min output from the PID.
	MinOutput float32
}

// TrackingControllerState holds the mutable state a TrackingController.
type TrackingControllerState struct {
	// ControlError is the difference between reference and current value.
	ControlError float32
	// ControlErrorIntegrand is the integrated control error over time.
	ControlErrorIntegrand float32
	// ControlErrorIntegral is the control error integrand integrated over time.
	ControlErrorIntegral float32
	// ControlErrorDerivative is the low-pass filtered time-derivative of the control error.
	ControlErrorDerivative float32
	// ControlSignal is the current control signal output of the controller.
	ControlSignal float32
	// UnsaturatedControlSignal is the control signal before saturation used for tracking the
	// actual control signal for bumpless transfer or compensation of un-modeled saturations.
	UnsaturatedControlSignal float32
}

// TrackingControllerInput holds the input parameters to a TrackingController.
type TrackingControllerInput struct {
	// ReferenceSignal is the reference value for the signal to control.
	ReferenceSignal float32
	// ActualSignal is the actual value of the signal to control.
	ActualSignal float32
	// FeedForwardSignal is the contribution of the feed-forward control loop in the controller output.
	FeedForwardSignal float32
	// AppliedControlSignal is the actual control command applied by the actuator.
	AppliedControlSignal float32
	// SamplingInterval is the time interval elapsed since the previous call of the controller Update method.
	SamplingInterval time.Duration
}

// Reset the controller state.
func (c *TrackingController) Reset() {
	c.State = TrackingControllerState{}
}

// Update the controller state.
func (c *TrackingController) Update(input TrackingControllerInput) {
	e := input.ReferenceSignal - input.ActualSignal
	controlErrorIntegral := c.State.ControlErrorIntegrand*seconds(input.SamplingInterval) + c.State.ControlErrorIntegral
	controlErrorDerivative := ((1/seconds(c.Config.LowPassTimeConstant))*(e-c.State.ControlError) +
		c.State.ControlErrorDerivative) / (seconds(input.SamplingInterval)/seconds(c.Config.LowPassTimeConstant) + 1)
	c.State.UnsaturatedControlSignal = e*c.Config.ProportionalGain + c.Config.IntegralGain*controlErrorIntegral +
		c.Config.DerivativeGain*controlErrorDerivative + input.FeedForwardSignal
	c.State.ControlSignal = max(c.Config.MinOutput, min(c.Config.MaxOutput, c.State.UnsaturatedControlSignal))
	c.State.ControlErrorIntegrand = e + c.Config.AntiWindUpGain*(input.AppliedControlSignal-
		c.State.UnsaturatedControlSignal)
	c.State.ControlErrorIntegrand = max(-MaxFloat32, min(MaxFloat32, c.State.ControlErrorIntegrand))
	c.State.ControlErrorIntegral = max(-MaxFloat32, min(MaxFloat32, controlErrorIntegral))
	c.State.ControlErrorDerivative = max(-MaxFloat32, min(MaxFloat32, controlErrorDerivative))
	c.State.ControlError = max(-MaxFloat32, min(MaxFloat32, e))
}

// DischargeIntegral provides the ability to discharge the controller integral state
// over a configurable period of time.
func (c *TrackingController) DischargeIntegral(dt time.Duration) {
	c.State.ControlErrorIntegrand = 0.0
	c.State.ControlErrorIntegral = max(
		0,
		min(1-seconds(dt)/c.Config.IntegralDischargeTimeConstant, 1.0),
	) * c.State.ControlErrorIntegral
}
