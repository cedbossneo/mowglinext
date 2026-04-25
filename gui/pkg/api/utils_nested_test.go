package api

import (
	"io"
	"strings"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/msgs/geometry"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestUnmarshalSetDockingPointReq(t *testing.T) {
	t.Run("snake_case docking_pose with floats", func(t *testing.T) {
		body := `{"docking_pose":{"position":{"x":4.2,"y":7.7,"z":0.0},"orientation":{"x":0,"y":0,"z":0.5,"w":0.866}}}`
		var req mowgli.SetDockingPointReq
		err := unmarshalROSMessage[*mowgli.SetDockingPointReq](io.NopCloser(strings.NewReader(body)), &req)
		require.NoError(t, err)
		assert.InDelta(t, 4.2, req.DockingPose.Position.X, 1e-9)
		assert.InDelta(t, 7.7, req.DockingPose.Position.Y, 1e-9)
		assert.InDelta(t, 0.0, req.DockingPose.Position.Z, 1e-9)
		assert.InDelta(t, 0.5, req.DockingPose.Orientation.Z, 1e-9)
		assert.InDelta(t, 0.866, req.DockingPose.Orientation.W, 1e-9)
	})

	t.Run("snake_case with x=1 only", func(t *testing.T) {
		body := `{"docking_pose":{"position":{"x":1.0,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":0}}}`
		var req mowgli.SetDockingPointReq
		err := unmarshalROSMessage[*mowgli.SetDockingPointReq](io.NopCloser(strings.NewReader(body)), &req)
		require.NoError(t, err)
		assert.InDelta(t, 1.0, req.DockingPose.Position.X, 1e-9)
	})

	t.Run("empty body", func(t *testing.T) {
		var req mowgli.SetDockingPointReq
		err := unmarshalROSMessage[*mowgli.SetDockingPointReq](io.NopCloser(strings.NewReader("{}")), &req)
		require.NoError(t, err)
		assert.Equal(t, geometry.Pose{}, req.DockingPose)
	})
}
